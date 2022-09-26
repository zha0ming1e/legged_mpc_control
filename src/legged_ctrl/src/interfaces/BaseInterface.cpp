
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <pinocchio/algorithm/frames.hpp>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>

#include "interfaces/BaseInterface.h"
#include "utils/Utils.h"

namespace legged
{


BaseInterface::BaseInterface(ros::NodeHandle &_nh, const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile) {
    nh = _nh;

    sub_joy_msg = nh.subscribe("/joy", 1000, &BaseInterface::joy_callback, this);

    // pinnochio stuff
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

    modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);


    // PinocchioInterface
    // TODO: there may be memory leak happen
    pinocchioInterfacePtr_.reset(
        new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));

    // CentroidalModelInfo
    centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
        *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
        centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
        modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);                


    CentroidalModelPinocchioMapping pinocchio_mapping(centroidalModelInfo_);
    ee_kinematics_ = std::unique_ptr<PinocchioEndEffectorKinematics>(new PinocchioEndEffectorKinematics(*pinocchioInterfacePtr_, pinocchio_mapping,
                                                modelSettings_.contactNames3DoF));
    // must set this manually                                                    
    ee_kinematics_->setPinocchioInterface(*pinocchioInterfacePtr_);


    pinocchio_state_q = vector_t(centroidalModelInfo_.generalizedCoordinatesNum); pinocchio_state_q.setZero();
    pinocchio_state_v = vector_t(centroidalModelInfo_.generalizedCoordinatesNum); pinocchio_state_v.setZero();

    wbc_ = std::make_shared<Wbc>(taskFile, *pinocchioInterfacePtr_, centroidalModelInfo_, *ee_kinematics_, swing_leg_ctrl_type, verbose);

    ik_solver = LeggedIKSolver::createLeggedIKSolver(taskFile, urdfFile, referenceFile);

    // joint position target helper variables
    pos_des = vector_t(centroidalModelInfo_.actuatedDofNum); pos_des.setZero();
    prev_pos_des = vector_t(centroidalModelInfo_.actuatedDofNum); prev_pos_des.setZero();
    vel_des = vector_t(centroidalModelInfo_.actuatedDofNum); vel_des.setZero();
}


void BaseInterface::
joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    // left updown
    legged_state.joy.velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

    //A
    if (joy_msg->buttons[0] == 1) {
        legged_state.joy.ctrl_state_change_request = true;
    }

    // right updown
    legged_state.joy.velx = joy_msg->axes[4] * JOY_CMD_VELX_MAX;
    // right horiz
    legged_state.joy.vely = joy_msg->axes[3] * JOY_CMD_VELY_MAX;
    // left horiz
    legged_state.joy.yaw_rate = joy_msg->axes[0] * JOY_CMD_YAW_MAX;
    // up-down button
    legged_state.joy.pitch_rate = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;
    // left-right button
    legged_state.joy.roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX;

    // lb
    if (joy_msg->buttons[4] == 1) {
        std::cout << "You have pressed the exit button!!!!" << std::endl;
        legged_state.joy.exit = true;
    }
}


bool BaseInterface::joy_update(double t, double dt) {
    if (legged_state.joy.exit) {
        return false;
    }

    // process joy cmd data to get desired height, velocity, yaw, etc
    // save the result into legged_state
    legged_state.joy.body_height += legged_state.joy.velz * dt;
    if (legged_state.joy.body_height >= JOY_CMD_BODY_HEIGHT_MAX) {
        legged_state.joy.body_height = JOY_CMD_BODY_HEIGHT_MAX;
    }
    if (legged_state.joy.body_height <= JOY_CMD_BODY_HEIGHT_MIN) {
        legged_state.joy.body_height = JOY_CMD_BODY_HEIGHT_MIN;
    }

    legged_state.joy.prev_ctrl_state = legged_state.joy.ctrl_state;

    if (legged_state.joy.ctrl_state_change_request) {
        // toggle legged_state.joy.ctrl_state
        legged_state.joy.ctrl_state = legged_state.joy.ctrl_state + 1;
        legged_state.joy.ctrl_state = legged_state.joy.ctrl_state % 2; //TODO: how to toggle more states?
        legged_state.joy.ctrl_state_change_request = false; //erase this change request;
    }
    return true;
}


bool BaseInterface::sensor_update(double t, double dt) {
    // calculate several useful variables
    // euler should be roll pitch yaw
    legged_state.fbk.root_rot_mat = legged_state.fbk.root_quat.toRotationMatrix();
    legged_state.fbk.root_euler = Utils::quat_to_euler(legged_state.fbk.root_quat);
    double yaw_angle = legged_state.fbk.root_euler[2];
    legged_state.fbk.root_ang_vel = legged_state.fbk.root_rot_mat * legged_state.fbk.imu_ang_vel;
    legged_state.fbk.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData();
    /* 
     * first only set joint configuration
     */
    pinocchio_state_q.setZero();
    pinocchio_state_v.setZero();
    pinocchio_state_q.tail(centroidalModelInfo_.actuatedDofNum) = legged_state.fbk.joint_pos;
    pinocchio_state_v.tail(centroidalModelInfo_.actuatedDofNum) = legged_state.fbk.joint_vel;
    
    pinocchio::forwardKinematics(model, data, pinocchio_state_q, pinocchio_state_v);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    std::vector<vector3_t> pos_measured = ee_kinematics_->getPosition(vector_t());
    std::vector<vector3_t> vel_measured = ee_kinematics_->getVelocity(vector_t(), vector_t());
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, centroidalModelInfo_.generalizedCoordinatesNum);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i)
    {
        pinocchio::getFrameJacobian(model, data, centroidalModelInfo_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        legged_state.fbk.foot_pos_rel.block<3, 1>(0, i) = pos_measured[i];
        legged_state.fbk.foot_vel_rel.block<3, 1>(0, i) = vel_measured[i];
        legged_state.fbk.j_foot.block<3, 3>(3 * i, 3 * i) = jac.block<3,3>(0, 6+3*i);
    }
    /* 
     * next set joint configuration and body pose/velocity to get world frame 
     */
    // pinnochio uses zyx euler angle 
    // pinocchio_state_q [position, euler, joint position]
    pinocchio_state_q.head<3>() = legged_state.fbk.root_pos;
    pinocchio_state_q.segment<3>(3) = Eigen::Vector3d(legged_state.fbk.root_euler[2], legged_state.fbk.root_euler[1], legged_state.fbk.root_euler[0]);

    pinocchio_state_v.head<3>() = legged_state.fbk.root_lin_vel;
    pinocchio_state_v.segment<3>(3) = getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(
      pinocchio_state_q.segment<3>(3), legged_state.fbk.imu_ang_vel);


    pinocchio::forwardKinematics(model, data, pinocchio_state_q, pinocchio_state_v);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    std::vector<vector3_t> pos_measured2 = ee_kinematics_->getPosition(vector_t());
    std::vector<vector3_t> vel_measured2 = ee_kinematics_->getVelocity(vector_t(), vector_t());
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i)
    {
        legged_state.fbk.foot_pos_world.block<3, 1>(0, i) = pos_measured2[i];
        legged_state.fbk.foot_vel_world.block<3, 1>(0, i) = vel_measured2[i];
    }

    estimation_update(t, dt);

    return true;
}

bool BaseInterface::estimation_update(double t, double dt) {
    // state estimation EKF
    if (!kf.is_inited()) {
        kf.init_state(legged_state);
    } else {
        kf.update_estimation(legged_state, dt);
    }
    return true;
}

bool BaseInterface::wbc_update(double t, double dt) {

    // assemble feedback into wbc's input measured_rbd_state
    vector_t measured_rbd_state = vector_t(2*centroidalModelInfo_.generalizedCoordinatesNum);
    measured_rbd_state.head<3>() = Eigen::Vector3d(legged_state.fbk.root_euler[2], legged_state.fbk.root_euler[1], legged_state.fbk.root_euler[0]);
    measured_rbd_state.segment<3>(3) = legged_state.fbk.root_pos;
    measured_rbd_state.segment(6, centroidalModelInfo_.actuatedDofNum) = legged_state.fbk.joint_pos;

    measured_rbd_state.segment<3>(centroidalModelInfo_.generalizedCoordinatesNum) = legged_state.fbk.root_ang_vel;
    measured_rbd_state.segment<3>(centroidalModelInfo_.generalizedCoordinatesNum + 3) = legged_state.fbk.root_lin_vel;
    measured_rbd_state.segment(centroidalModelInfo_.generalizedCoordinatesNum + 6, centroidalModelInfo_.actuatedDofNum) = legged_state.fbk.joint_vel;

    size_t planned_mode = stanceLeg2ModeNumber(legged_state.ctrl.plan_contacts);

    // optimized_state optimized_input should contain world frame foot position velocity target 

    vector_t x = wbc_->update(legged_state.ctrl.optimized_state, legged_state.ctrl.optimized_input, 
        measured_rbd_state, planned_mode);

    vector_t torque = x.tail(12);

    if (swing_leg_ctrl_type == 0) {
        pos_des = centroidal_model::getJointAngles(legged_state.ctrl.optimized_state, centroidalModelInfo_);
        vel_des = centroidal_model::getJointVelocities(legged_state.ctrl.optimized_input, centroidalModelInfo_);
    } else {
        ik_solver -> setBasePos(legged_state.ctrl.optimized_state.head(6)); 
        for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i) {
            pos_des.segment<3>(3*i) = ik_solver->solveIK(legged_state.ctrl.optimized_state.segment<3>(6+3*i), i);
            // vel uses finite diff
            vel_des.segment<3>(3*i) = (pos_des.segment<3>(3*i) - prev_pos_des.segment<3>(3*i)) / dt;
            prev_pos_des.segment<3>(3*i) = pos_des.segment<3>(3*i);
        }
    }

    // save command to struct
    legged_state.ctrl.joint_ang_tgt = pos_des;
    legged_state.ctrl.joint_vel_tgt = vel_des;
    legged_state.ctrl.joint_tau_tgt = torque;

    return true;
}

}  // namespace legged