
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include "utils/LeggedIKSolver.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

// Boost
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace legged
{
    std::unique_ptr<LeggedIKSolver> LeggedIKSolver::createLeggedIKSolver(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile) {
        bool verbose = true;
        // check that task file exists
        boost::filesystem::path task_file_path(taskFile);
        if (boost::filesystem::exists(task_file_path))
            std::cerr << "[LeggedInterface] Loading task file: " << task_file_path << std::endl;
        else
            throw std::invalid_argument("[LeggedInterface] Task file not found: " + task_file_path.string());

        // check that urdf file exists
        boost::filesystem::path urdf_file_path(urdfFile);
        if (boost::filesystem::exists(urdf_file_path))
            std::cerr << "[LeggedInterface] Loading Pinocchio model from: " << urdf_file_path << std::endl;
        else
            throw std::invalid_argument("[LeggedInterface] URDF file not found: " + urdf_file_path.string());

        // TODO: there may be memory leak happen
        ModelSettings* modelSettings_ = new ModelSettings(loadModelSettings(taskFile, "model_settings", verbose));


        
        PinocchioInterface* pinocchioInterface = new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_->jointNames));


        // CentroidalModelInfo
        CentroidalModelInfo* centroidalModelInfo_=  new CentroidalModelInfo(centroidal_model::createCentroidalModelInfo(
            *pinocchioInterface, centroidal_model::loadCentroidalType(taskFile),
            centroidal_model::loadDefaultJointState(pinocchioInterface->getModel().nq - 6, referenceFile),
            modelSettings_->contactNames3DoF, modelSettings_->contactNames6DoF));                


        CentroidalModelPinocchioMapping* pinocchio_mapping = new CentroidalModelPinocchioMapping(*centroidalModelInfo_);
        PinocchioEndEffectorKinematics* ee_kinematics = new PinocchioEndEffectorKinematics(*pinocchioInterface, *pinocchio_mapping,
                                                    modelSettings_->contactNames3DoF);
        // must set this manually                                                    
        ee_kinematics -> setPinocchioInterface(*pinocchioInterface);

        return std::unique_ptr<LeggedIKSolver>(new LeggedIKSolver(*pinocchioInterface, *centroidalModelInfo_, *ee_kinematics));    
    }


    LeggedIKSolver::LeggedIKSolver(PinocchioInterface& pino_interface, CentroidalModelInfo& info,
                const PinocchioEndEffectorKinematics& ee_kinematics) 
    : pino_interface_(pino_interface)
    , info_(info)
    , centroidal_dynamics_(info_)
    , mapping_(info_)
    , ee_kinematics_(ee_kinematics.clone())
    {
        centroidal_dynamics_.setPinocchioInterface(pino_interface_);
        mapping_.setPinocchioInterface(pino_interface_);
        ee_kinematics_->setPinocchioInterface(pino_interface_); 

        // setup vectors
        state_q = vector_t(info_.generalizedCoordinatesNum); state_q.setZero();
        state_v = vector_t(info_.generalizedCoordinatesNum); state_v.setZero();
        joint_angs4_ = vector_t(info_.actuatedDofNum);       joint_angs4_.setZero();
        prev_joint_angs4_ = vector_t(info_.actuatedDofNum);  prev_joint_angs4_.setZero();
        foot_pos4_ = vector_t(info_.actuatedDofNum);         foot_pos4_.setZero();
    }


    void LeggedIKSolver::setWarmStartPos(vector_t prev_joint_angs, int leg_id) {
        // TODO: make sure leg_id is 0-3
        prev_joint_angs4_.segment<3>(3*leg_id) = prev_joint_angs;
    }


    void LeggedIKSolver::setBasePos(vector_t orientation_pos) {
        state_q.head(6) = orientation_pos;
    }



    vector_t LeggedIKSolver::solveIK(vector3_t foot_pos, int leg_id) {

        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        Eigen::Matrix<scalar_t, 3, 3> pos_jac;
        Eigen::Matrix<scalar_t, 3, 3> pos_jjt;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        std::vector<vector3_t> pos_measured;
        Eigen::Vector3d pos_err;
        bool success = false;
        const auto& model = pino_interface_.getModel();
        auto& data = pino_interface_.getData();
        Eigen::VectorXd v(model.nv); v.setZero();

        // get warm start angle 
        state_q.segment<3>(6+3*leg_id) = prev_joint_angs4_.segment<3>(3*leg_id);

        for (int i=0;i<IT_MAX;i++) {
            pinocchio::forwardKinematics(model, data, state_q, state_v);
            pinocchio::updateFramePlacements(model, data);
            pinocchio::computeJointJacobians(model, data);
            pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[leg_id],  
                pinocchio::LOCAL_WORLD_ALIGNED, jac);
            pos_jac = jac.block<3,3>(0, 6+3*leg_id);
            pos_measured = ee_kinematics_->getPosition(vector_t());

            // std::cout << "angle: " << state_q.segment<3>(6+3*leg_id).transpose() 
            // << " fk: " << pos_measured[leg_id].transpose() << std::endl;
            pos_err = foot_pos - pos_measured[leg_id];
            // std::cout << pos_err << std::endl;
            if(pos_err.norm() < EPS)
            {
                success = true;
                break;
            }
            if (i >= IT_MAX)
            {
            success = false;
            break;
            }
            pos_jjt.noalias() = pos_jac * pos_jac.transpose();
            pos_jjt.diagonal().array() += DAMP;
            v.segment<3>(6+3*leg_id) = pos_jac.transpose() * pos_jjt.ldlt().solve(pos_err);
            state_q = pinocchio::integrate(model,state_q,v*DT);

        }
        // std::cout << "\nresult: " << state_q.segment<3>(6+3*leg_id).transpose() << std::endl;
        // std::cout << "\nfinal error: " << pos_err.transpose() << std::endl;
        if(success) 
        {
            // std::cout << "Convergence achieved!" << std::endl;
            prev_joint_angs4_.segment<3>(3*leg_id) = state_q.tail(info_.actuatedDofNum);
        }
        else 
        {
            std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
        }
        return state_q.segment<3>(6+3*leg_id);        
    }


} // namespace legged
