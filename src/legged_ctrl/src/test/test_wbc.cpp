
#include "wbc_ctrl/wbc.h"
#include "utils/LeggedIKSolver.h"


#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/MultipleShootingSettings.h>

#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <ocs2_legged_robot/LeggedRobotPreComputation.h>
#include <ocs2_legged_robot/constraint/FrictionConeConstraint.h>
#include <ocs2_legged_robot/constraint/NormalVelocityConstraintCppAd.h>
#include <ocs2_legged_robot/constraint/ZeroForceConstraint.h>
#include <ocs2_legged_robot/constraint/ZeroVelocityConstraintCppAd.h>
#include <ocs2_legged_robot/cost/LeggedRobotQuadraticTrackingCost.h>
#include <ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h>
#include <ocs2_legged_robot/initialization/LeggedRobotInitializer.h>

using namespace legged;
using namespace ocs2;
using namespace legged_robot;

int main() {

    // Initialize OCS2
    std::string taskFile = "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/config/task.info", 
                urdfFile = "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/urdf/a1_description/urdf/a1.urdf",
                referenceFile = "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/config/reference.info";

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

    ModelSettings modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);


    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
    CentroidalModelInfo centroidalModelInfo_;
    // PinocchioInterface
    pinocchioInterfacePtr_.reset(
        new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));


    // CentroidalModelInfo
    centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
        *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
        centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
        modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);                


    CentroidalModelPinocchioMapping pinocchio_mapping(centroidalModelInfo_);
    PinocchioEndEffectorKinematics ee_kinematics(*pinocchioInterfacePtr_, pinocchio_mapping,
                                                modelSettings_.contactNames3DoF);
    // must set this manually                                                    
    ee_kinematics.setPinocchioInterface(*pinocchioInterfacePtr_);

    std::shared_ptr<Wbc> wbc_;
    // state desired and input desired contains WORK SPACE target 
    wbc_ = std::make_shared<Wbc>(taskFile, *pinocchioInterfacePtr_, centroidalModelInfo_, ee_kinematics, 1, verbose);

    vector_t measured_rbd_state = vector_t(2*centroidalModelInfo_.generalizedCoordinatesNum);
    vector_t optimized_state = vector_t(centroidalModelInfo_.generalizedCoordinatesNum);
    vector_t optimized_input = vector_t(3 * centroidalModelInfo_.numThreeDofContacts + 
                                        6 * centroidalModelInfo_.numSixDofContacts + 
                                            centroidalModelInfo_.actuatedDofNum);

    // need to fill some test data here  
    measured_rbd_state << 0, 0, 0,    
                          0, 0, 0.0,
                          -0.05, 0.72, -1.44,
                          -0.05, 0.72, -1.44,
                          0.05, 0.72, -1.44,
                          0.05, 0.72, -1.44,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0;

    optimized_state <<    0, 0, 0,    
                          0, 0, 0.3,    
                          0.18,  0.15, -0.32,
                         -0.18,  0.15, -0.32,            
                          0.18, -0.15, -0.32,            
                         -0.18, -0.15, -0.32;
    optimized_input <<  0, 0, centroidalModelInfo_.robotMass*9.8/4,
                        0, 0, centroidalModelInfo_.robotMass*9.8/4,
                        0, 0, centroidalModelInfo_.robotMass*9.8/4,
                        0, 0, centroidalModelInfo_.robotMass*9.8/4,
                        0, 0, 0,  
                        0, 0, 0,  
                        0, 0, 0,  
                        0, 0, 0;                                  


    int planned_mode = 15; // all on the ground
    //
    vector_t x = wbc_->update(optimized_state, optimized_input, measured_rbd_state, planned_mode);

    vector_t torque = x.tail(12);

    // test ik
    LeggedIKSolver ik_solve(*pinocchioInterfacePtr_, centroidalModelInfo_, ee_kinematics);
    int leg_id = 0;
    ik_solve.setWarmStartPos(measured_rbd_state.segment<3>(6, 3*leg_id), leg_id);

    // test pinocchio kinematics 
    vector_t  measured_q_, measured_v_;
    measured_q_ = vector_t(centroidalModelInfo_.generalizedCoordinatesNum);
    measured_v_ = vector_t(centroidalModelInfo_.generalizedCoordinatesNum);
    measured_q_.head<3>() = measured_rbd_state.segment<3>(3);
    measured_q_.segment<3>(3) = measured_rbd_state.head<3>();
    measured_q_.tail(centroidalModelInfo_.actuatedDofNum) = measured_rbd_state.segment(6, centroidalModelInfo_.actuatedDofNum);

    measured_v_.head<3>() = measured_rbd_state.segment<3>(centroidalModelInfo_.generalizedCoordinatesNum + 3);
    measured_v_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
        measured_q_.segment<3>(3), measured_rbd_state.segment<3>(centroidalModelInfo_.generalizedCoordinatesNum));
    measured_v_.tail(centroidalModelInfo_.actuatedDofNum) =
        measured_rbd_state.segment(centroidalModelInfo_.generalizedCoordinatesNum + 6, centroidalModelInfo_.actuatedDofNum);

    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData();
    pinocchio::forwardKinematics(model, data, measured_q_, measured_v_);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);

    // the angle
    std::vector<vector3_t> pos_measured = ee_kinematics.getPosition(vector_t());
    std::cout << "pos_measured"<< std::endl << pos_measured[0] << std::endl;


    measured_rbd_state << 0, 0, 0,    
                          0, 0, 0.0,
                          -0.05, 0.78, -1.44,
                          -0.05, 0.78, -1.44,
                          0.05, 0.78, -1.44,
                          0.05, 0.78, -1.44,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0;
    measured_q_.tail(centroidalModelInfo_.actuatedDofNum) = measured_rbd_state.segment(6, centroidalModelInfo_.actuatedDofNum);
    pinocchio::forwardKinematics(model, data, measured_q_, measured_v_);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);

    // the angle
    pos_measured = ee_kinematics.getPosition(vector_t());
    std::cout << "pos_measured2"<< std::endl << pos_measured[0] << std::endl;

    for (int i = 0; i < 10; i++) {
        vector_t angle = ik_solve.solveIK(pos_measured[0], leg_id);
        std::cout << "angle solved"<< std::endl << angle << std::endl;
        measured_q_.segment<3>(6+3*leg_id) = angle;
        pinocchio::forwardKinematics(model, data, measured_q_, measured_v_);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::computeJointJacobians(model, data);
        pos_measured = ee_kinematics.getPosition(vector_t());
        std::cout << "pos_measured"<< std::endl << pos_measured[0] << std::endl;
    }

    // if wbc uses workspace, now we need ik here to calculate joint angles11
    // vector_t pos_des = centroidal_model::getJointAngles(optimized_state, centroidalModelInfo_);
    // vector_t vel_des = centroidal_model::getJointVelocities(optimized_input, centroidalModelInfo_);

    // std::cout << "torque" << torque << std::endl;
    // std::cout << "pos_des" << pos_des << std::endl;
    // std::cout << "vel_des" << vel_des << std::endl;

    return 0;
}