
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

    // old a1 kinematics
    // leg order: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.1805;
    leg_offset_x[1] = 0.1805;
    leg_offset_x[2] = -0.1805;
    leg_offset_x[3] = -0.1805;
    leg_offset_y[0] = 0.047;
    leg_offset_y[1] = -0.047;
    leg_offset_y[2] = 0.047;
    leg_offset_y[3] = -0.047;
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.21;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = LOWER_LEG_LENGTH;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    // load parameter is very important 
    legged_state.param.load(_nh);
}


void BaseInterface::
joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    // left updown
    legged_state.joy.velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

    //A
    if (joy_msg->buttons[0] == 1) {
        std::cout << std::endl << "You have requested to chaneg legged_state!" << std::endl << std::endl;
        legged_state.joy.ctrl_state_change_request = true;
    }

    // xbox controller mapping
    // right updown
    legged_state.joy.velx = joy_msg->axes[4] * JOY_CMD_VELX_MAX;
    // right horiz
    legged_state.joy.vely = joy_msg->axes[3] * JOY_CMD_VELY_MAX;
    // left horiz
    legged_state.joy.yaw_rate = joy_msg->axes[0] * JOY_CMD_YAW_MAX;
    // up-down button
    // legged_state.joy.pitch_rate = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;
    // left-right button
    // legged_state.joy.roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX;

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

    // update movement mode
    if (legged_state.joy.ctrl_state == 1) {
        // walking mode, in this mode the robot should execute gait
        legged_state.ctrl.movement_mode = 1;
    } else if (legged_state.joy.ctrl_state == 0 && legged_state.joy.prev_ctrl_state == 1) {
        // leave walking mode
        // lock current position, should just happen for one instance
        legged_state.ctrl.movement_mode = 0;
    } else {
        legged_state.ctrl.movement_mode = 0;
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

    // const auto& model = pinocchioInterfacePtr_->getModel();
    // auto& data = pinocchioInterfacePtr_->getData();
    // /* 
    //  * first only set joint configuration
    //  */
    // pinocchio_state_q.setZero();
    // pinocchio_state_v.setZero();
    // pinocchio_state_q.tail(centroidalModelInfo_.actuatedDofNum) = Utils::joint_vec_unitree_to_pinnochio(legged_state.fbk.joint_pos);
    // pinocchio_state_v.tail(centroidalModelInfo_.actuatedDofNum) = Utils::joint_vec_unitree_to_pinnochio(legged_state.fbk.joint_vel);

    // // always warm start with the feedback joint position
    // ik_solver -> setWarmStartPos(Utils::joint_vec_unitree_to_pinnochio(legged_state.fbk.joint_pos));
    
    // pinocchio::forwardKinematics(model, data, pinocchio_state_q, pinocchio_state_v);
    // pinocchio::computeJointJacobians(model, data);
    // pinocchio::updateFramePlacements(model, data);
    // std::vector<vector3_t> pos_measured = ee_kinematics_->getPosition(vector_t());
    // std::vector<vector3_t> vel_measured = ee_kinematics_->getVelocity(vector_t(), vector_t());
    // Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    // jac.setZero(6, centroidalModelInfo_.generalizedCoordinatesNum);
    // for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i)
    // {
    //     pinocchio::getFrameJacobian(model, data, centroidalModelInfo_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    //     legged_state.fbk.foot_pos_rel.block<3, 1>(0, i) = pos_measured[i];
    //     legged_state.fbk.foot_vel_rel.block<3, 1>(0, i) = vel_measured[i];
    // }
    // // swap because of joint order! refer to src/legged_ctrl/include/utils/Utils.h
    // legged_state.fbk.j_foot.block<3, 3>(3 * 0, 3 * 0) = jac.block<3,3>(0,6+3*0);
    // legged_state.fbk.j_foot.block<3, 3>(3 * 1, 3 * 1) = jac.block<3,3>(0,6+3*2);
    // legged_state.fbk.j_foot.block<3, 3>(3 * 2, 3 * 2) = jac.block<3,3>(0,6+3*1);
    // legged_state.fbk.j_foot.block<3, 3>(3 * 3, 3 * 3) = jac.block<3,3>(0,6+3*3);

    // /* 
    //  * next set joint configuration and body pose/velocity to get world frame 
    //  */
    // // pinnochio uses zyx euler angle, we use Utils::quatToZyx 
    // // pinocchio_state_q [position, euler, joint position]
    // pinocchio_state_q.head<3>() = legged_state.fbk.root_pos;
    // pinocchio_state_q.segment<3>(3) = Utils::quatToZyx(legged_state.fbk.root_quat);

    // pinocchio_state_v.head<3>() = legged_state.fbk.root_lin_vel;
    // pinocchio_state_v.segment<3>(3) = getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(
    //   pinocchio_state_q.segment<3>(3), legged_state.fbk.imu_ang_vel);


    // pinocchio::forwardKinematics(model, data, pinocchio_state_q, pinocchio_state_v);
    // pinocchio::computeJointJacobians(model, data);
    // pinocchio::updateFramePlacements(model, data);
    // std::vector<vector3_t> pos_measured2 = ee_kinematics_->getPosition(vector_t());
    // std::vector<vector3_t> vel_measured2 = ee_kinematics_->getVelocity(vector_t(), vector_t());
    // for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i)
    // {
    //     legged_state.fbk.foot_pos_world.block<3, 1>(0, i) = pos_measured2[i];
    //     legged_state.fbk.foot_vel_world.block<3, 1>(0, i) = vel_measured2[i];
    //     legged_state.fbk.foot_pos_abs.block<3, 1>(0, i) = legged_state.fbk.foot_pos_world.block<3, 1>(0, i) - legged_state.fbk.root_pos;
    //     legged_state.fbk.foot_vel_abs.block<3, 1>(0, i) = legged_state.fbk.foot_vel_world.block<3, 1>(0, i) - legged_state.fbk.root_lin_vel;
    // }


    // use old a1 kinematics
    // FL, FR, RL, RR
    // std::cout << "---------------------------------" << std::endl;
    for (int i = 0; i < NUM_LEG; ++i) {
        legged_state.fbk.foot_pos_rel.block<3, 1>(0, i) = a1_kin.fk(
                legged_state.fbk.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        Eigen::Matrix3d jac = a1_kin.jac(
                legged_state.fbk.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        // std::cout << jac << " ---jac"<<i<<"--- " << legged_state.fbk.j_foot.block<3, 3>(3 * i, 3 * i) << std::endl;
        legged_state.fbk.j_foot.block<3, 3>(3 * i, 3 * i) = jac;
        Eigen::Matrix3d tmp_mtx = legged_state.fbk.j_foot.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = legged_state.fbk.joint_vel.segment<3>(3 * i);
        legged_state.fbk.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

        legged_state.fbk.foot_pos_abs.block<3, 1>(0, i) = legged_state.fbk.root_rot_mat * legged_state.fbk.foot_pos_rel.block<3, 1>(0, i);
        legged_state.fbk.foot_vel_abs.block<3, 1>(0, i) = legged_state.fbk.root_rot_mat * legged_state.fbk.foot_vel_rel.block<3, 1>(0, i);

        legged_state.fbk.foot_pos_world.block<3, 1>(0, i) = legged_state.fbk.foot_pos_abs.block<3, 1>(0, i) + legged_state.fbk.root_pos;
        legged_state.fbk.foot_vel_world.block<3, 1>(0, i) = legged_state.fbk.foot_vel_abs.block<3, 1>(0, i) + legged_state.fbk.root_lin_vel;
    }
    // std::cout << "---------------------------------" << std::endl;
    // //compare
    // std::cout << "---------------------------------" << std::endl;
    // std::cout << legged_state.fbk.foot_pos_world.block<3, 1>(0, 0).transpose() << " ---FL pos_world0 LF--- " << pos_measured2[0].transpose() << std::endl;
    // std::cout << legged_state.fbk.foot_pos_world.block<3, 1>(0, 1).transpose() << " ---pos_world1--- " << pos_measured2[1].transpose() << std::endl;
    // std::cout << legged_state.fbk.foot_pos_world.block<3, 1>(0, 2).transpose() << " ---pos_world2--- " << pos_measured2[2].transpose() << std::endl;
    // std::cout << legged_state.fbk.foot_pos_world.block<3, 1>(0, 3).transpose() << " ---pos_world3--- " << pos_measured2[3].transpose() << std::endl;
    // std::cout << legged_state.fbk.foot_pos_rel.block<3, 1>(0, 0).transpose() << " ---pos_rel0--- " << pos_measured[0].transpose() << std::endl;
    // std::cout << legged_state.fbk.foot_pos_rel.block<3, 1>(0, 1).transpose() << " ---pos_rel1--- " << pos_measured[1].transpose() << std::endl;
    // std::cout << legged_state.fbk.foot_pos_rel.block<3, 1>(0, 2).transpose() << " ---pos_rel2--- " << pos_measured[2].transpose() << std::endl;
    // std::cout << legged_state.fbk.foot_pos_rel.block<3, 1>(0, 3).transpose() << " ---pos_rel3--- " << pos_measured[3].transpose() << std::endl;
    // std::cout << "---------------------------------" << std::endl;

    // a dynamic model for foot contact thresholding
    // TODO: make some parameters configurable
    for (int i = 0; i < NUM_LEG; ++i) {
           double force_mag = legged_state.fbk.foot_force[i];

            if (force_mag < legged_state.fbk.foot_force_min[i])
            {
                legged_state.fbk.foot_force_min[i] = 0;
            }
            if (force_mag > legged_state.fbk.foot_force_max[i])
            {
                legged_state.fbk.foot_force_max[i] = 300;
            }
            // exponential decay, max force decays faster
            // legged_state.fbk.foot_force_min[i] *= 0.9991;
            // legged_state.fbk.foot_force_max[i] *= 0.997;
            legged_state.fbk.foot_force_contact_threshold[i] = 
                legged_state.fbk.foot_force_min[i] + 0.5 * (legged_state.fbk.foot_force_max[i] - legged_state.fbk.foot_force_min[i]);

            legged_state.fbk.foot_contact_flag[i] = 
                1.0 / (1 + exp(-10 * (force_mag - legged_state.fbk.foot_force_contact_threshold[i])));        
    }


    estimation_update(t, dt);


    // always calculate Raibert Heuristic, calculate foothold position
    // update foot plan: legged_state.foot_pos_target_world
    Eigen::Vector3d lin_vel_abs = legged_state.fbk.root_lin_vel; lin_vel_abs[2] = 0;// world frame linear velocity, cannot regulate z velocity so we set it to 0
    // desired abs frame linear velocity
    Eigen::Vector3d lin_vel_d_abs = legged_state.fbk.root_rot_mat_z * legged_state.ctrl.root_lin_vel_d_rel; // robot body frame linear velocity

    // foothold target 
    legged_state.ctrl.foot_pos_target_abs = legged_state.fbk.root_rot_mat_z * legged_state.param.default_foot_pos_rel;
    for (int i = 0; i < NUM_LEG; ++i) {
        double delta_x =
                std::sqrt(std::abs(legged_state.param.default_foot_pos_rel(2)) / 9.8) * (lin_vel_abs(0) - lin_vel_d_abs(0)) +
                (1.0/legged_state.param.gait_counter_speed/2.0) / 2.0 *
                lin_vel_d_abs(0);
        double delta_y =
                std::sqrt(std::abs(legged_state.param.default_foot_pos_rel(2)) / 9.8) * (lin_vel_abs(1) - lin_vel_d_abs(1)) +
                (1.0/legged_state.param.gait_counter_speed/2.0) / 2.0 *
                lin_vel_d_abs(1);

        if (delta_x < -FOOT_DELTA_X_LIMIT) {
            delta_x = -FOOT_DELTA_X_LIMIT;
        }
        if (delta_x > FOOT_DELTA_X_LIMIT) {
            delta_x = FOOT_DELTA_X_LIMIT;
        }
        if (delta_y < -FOOT_DELTA_Y_LIMIT) {
            delta_y = -FOOT_DELTA_Y_LIMIT;
        }
        if (delta_y > FOOT_DELTA_Y_LIMIT) {
            delta_y = FOOT_DELTA_Y_LIMIT;
        }

        // modify abs frame foothold target
        legged_state.ctrl.foot_pos_target_abs(0, i) += delta_x;
        legged_state.ctrl.foot_pos_target_abs(1, i) += delta_y;

        // dont really use this but still calculate it
        legged_state.ctrl.foot_pos_target_rel.block<3, 1>(0, i) = legged_state.fbk.root_rot_mat.transpose() * legged_state.ctrl.foot_pos_target_abs.block<3, 1>(0, i);

        // swing leg need to use foot_pos_target_world
        legged_state.ctrl.foot_pos_target_world.block<3, 1>(0, i) = legged_state.ctrl.foot_pos_target_abs.block<3, 1>(0, i) + legged_state.fbk.root_pos;
    }

    return true;
}

bool BaseInterface::estimation_update(double t, double dt) {
    // legged_state estimation EKF
    if (!kf.is_inited()) {
        kf.init_state(legged_state);
    } else {
        kf.update_estimation(legged_state, dt);
    }
    return true;
}

bool BaseInterface::tau_ctrl_update(double t, double dt) {

    // test old control strategy
    for(int i = 0; i < NUM_LEG; ++i) {
        // Ground reaction forces assignment, world to body
        foot_forces_grf_rel.block<3,1>(0,i) = legged_state.fbk.root_rot_mat.transpose() * 
            legged_state.ctrl.optimized_input.segment<3>(i*3); 

        Eigen::Matrix3d jac = legged_state.fbk.j_foot.block<3,3>(3*i, 3*i); 
        legged_state.ctrl.joint_tau_tgt.segment<3>(i*3) = -jac.transpose() * foot_forces_grf_rel.block<3,1>(0,i);  
        // TODO: add dynamics feedforward

        if (legged_state.ctrl.movement_mode > 0) {
            // foot target pos/vel assignment
            foot_pos_target_rel.block<3, 1>(0, i) = legged_state.fbk.root_rot_mat.transpose() * 
                (legged_state.ctrl.optimized_state.segment<3>(6 + 3 * i)  - legged_state.fbk.root_pos);


            foot_vel_target_rel.block<3, 1>(0, i) = legged_state.fbk.root_rot_mat.transpose() * 
                (legged_state.ctrl.optimized_input.segment<3>(12 + 3 * i)  - legged_state.fbk.root_lin_vel);

            Eigen::Vector3d joint_ang_tgt = a1_kin.inv_kin(foot_pos_target_rel.block<3, 1>(0, i), legged_state.fbk.joint_pos.segment<3>(i*3), rho_opt_list[i], rho_fix_list[i]);
            if ((isnan(joint_ang_tgt[0])) || (isnan(joint_ang_tgt[1])) || (isnan(joint_ang_tgt[2]))) {
                legged_state.ctrl.prev_joint_ang_tgt.segment<3>(i*3) = legged_state.ctrl.joint_ang_tgt.segment<3>(i*3);
                legged_state.ctrl.joint_ang_tgt.segment<3>(i*3) = legged_state.fbk.joint_pos.segment<3>(i*3);
            } else {
                legged_state.ctrl.prev_joint_ang_tgt.segment<3>(i*3) = legged_state.ctrl.joint_ang_tgt.segment<3>(i*3);
                legged_state.ctrl.joint_ang_tgt.segment<3>(i*3) = joint_ang_tgt;
            }
            Eigen::Vector3d joint_vel_tgt = jac.lu().solve(foot_vel_target_rel.block<3, 1>(0, i));
            if ((isnan(joint_vel_tgt[0])) || (isnan(joint_vel_tgt[1])) || (isnan(joint_vel_tgt[2]))) {
                legged_state.ctrl.joint_vel_tgt.segment<3>(i*3) = legged_state.fbk.joint_vel.segment<3>(i*3);
            } else {
                legged_state.ctrl.joint_vel_tgt.segment<3>(i*3) = joint_vel_tgt;
            }
        }   else {
            legged_state.ctrl.prev_joint_ang_tgt.segment<3>(i*3) = legged_state.fbk.joint_pos.segment<3>(i*3);
            legged_state.ctrl.joint_ang_tgt.segment<3>(i*3) = legged_state.fbk.joint_pos.segment<3>(i*3);
            legged_state.ctrl.joint_vel_tgt.segment<3>(i*3) = legged_state.fbk.joint_vel.segment<3>(i*3);
        }

    } 
    
    // std::cout << "foot_pos_target_rel: " << foot_pos_target_rel.transpose() << std::endl;
    // std::cout << "joint_pos: " << legged_state.fbk.joint_pos.transpose() << std::endl;
    // std::cout << "joint_ang_tgt: " << legged_state.ctrl.joint_ang_tgt.transpose() << std::endl;
    
    // legged_state.ctrl.joint_ang_tgt = legged_state.fbk.joint_pos;
    // legged_state.ctrl.joint_vel_tgt = legged_state.fbk.joint_vel;
    
    return true;
}

bool BaseInterface::wbc_update(double t, double dt) {

    // TODO:
    if (legged_state.mpc_solver_inited == false) {
        std::cout << "mpc_solver is not generating results!" <<std::endl;
        return true;
    }

    // assemble feedback into wbc's input measured_rbd_state
    vector_t measured_rbd_state = vector_t(2*centroidalModelInfo_.generalizedCoordinatesNum);
    measured_rbd_state.head<3>() = Utils::quatToZyx(legged_state.fbk.root_quat);
    measured_rbd_state.segment<3>(3) = legged_state.fbk.root_pos;
    measured_rbd_state.segment(6, centroidalModelInfo_.actuatedDofNum) = Utils::joint_vec_unitree_to_pinnochio(legged_state.fbk.joint_pos);

    measured_rbd_state.segment<3>(centroidalModelInfo_.generalizedCoordinatesNum) = legged_state.fbk.root_ang_vel;
    measured_rbd_state.segment<3>(centroidalModelInfo_.generalizedCoordinatesNum + 3) = legged_state.fbk.root_lin_vel;
    measured_rbd_state.segment(centroidalModelInfo_.generalizedCoordinatesNum + 6, centroidalModelInfo_.actuatedDofNum) = Utils::joint_vec_unitree_to_pinnochio(legged_state.fbk.joint_vel);

    // convert legged_state.ctrl.plan_contacts to std::array<bool, NUM_LEG>
    std::array<bool, NUM_LEG> tmp = {legged_state.ctrl.plan_contacts[0], legged_state.ctrl.plan_contacts[1], legged_state.ctrl.plan_contacts[2], legged_state.ctrl.plan_contacts[3]}; 
    size_t planned_mode = stanceLeg2ModeNumber(tmp);

    // optimized_state optimized_input should contain world frame foot position velocity target 

    vector_t x = wbc_->update(legged_state.ctrl.optimized_state, legged_state.ctrl.optimized_input, 
        measured_rbd_state, planned_mode);

    Eigen::Matrix<scalar_t, 12, 1> torque = x.tail(12);

    if (swing_leg_ctrl_type == 0) {
        // TODO: check joint orders
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
    legged_state.ctrl.joint_tau_tgt = Utils::joint_vec_pinnochio_to_unitree(torque);

    std::cout << "measured_rbd_state \t " << measured_rbd_state.segment(0, centroidalModelInfo_.generalizedCoordinatesNum).transpose() << std::endl;
    std::cout << "optimized_state \t " << legged_state.ctrl.optimized_state.transpose() << std::endl;
    std::cout << "optimized_input \t " << legged_state.ctrl.optimized_input.transpose() << std::endl;
    std::cout << "pos_des \t " << pos_des.transpose() << std::endl;
    std::cout << "vel_des \t " << vel_des.transpose() << std::endl;
    std::cout << "torque \t " << torque.transpose() << std::endl;
    return true;
}

}  // namespace legged