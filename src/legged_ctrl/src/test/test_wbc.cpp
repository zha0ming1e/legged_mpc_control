
#include "wbc_ctrl/wbc.h"


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

    std::shared_ptr<Wbc> wbc_;
    wbc_ = std::make_shared<Wbc>(taskFile, *pinocchioInterfacePtr_, centroidalModelInfo_, ee_kinematics, verbose);
    return 0;
}