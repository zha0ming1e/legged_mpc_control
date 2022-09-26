/*
 *  Important shared variables between the robot interface and the cotroller 
 */

#pragma once
#include <Eigen/Dense>
#include <ros/ros.h>

#include "LeggedParams.h"

namespace legged
{
class LeggedFeedback {
    public:
        LeggedFeedback() {
            reset();
        }
    void reset();


    // IMU sensor data
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_ang_vel;


    // important feedback kinematics variables
    Eigen::Vector3d root_pos;
    Eigen::Quaterniond root_quat;
    Eigen::Vector3d root_euler;     // here we use roll, pitch, yaw
    Eigen::Matrix3d root_rot_mat;
    Eigen::Matrix3d root_rot_mat_z;
    Eigen::Vector3d root_lin_vel;
    Eigen::Vector3d root_ang_vel;
    Eigen::Vector3d root_acc;

    Eigen::Vector4d foot_force;
    Eigen::Matrix<double, NUM_DOF, 1> joint_pos;
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel;

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs;   // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel;   // in the robot frame

    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_world;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_rel;
    Eigen::Matrix<double, 12, 12> j_foot;

 
    // state estimation result
    bool estimated_contacts[NUM_LEG];  // true if the estimator thinks the foot has contact
    Eigen::Vector3d estimated_root_pos;
    Eigen::Vector3d estimated_root_vel;
};

class LeggedCtrl {
    public:
        LeggedCtrl() {
            reset();
        }
    void reset();


    Eigen::Vector4d gait_counter;          // use time directly
    Eigen::Vector4d gait_counter_speed;    // delta time

    // control target
    Eigen::Vector3d root_pos_d;
    Eigen::Vector3d root_euler_d;
    Eigen::Vector3d root_lin_vel_d_rel;
    Eigen::Vector3d root_lin_vel_d_world;
    Eigen::Vector3d root_ang_vel_d_rel;
    Eigen::Vector3d root_ang_vel_d_world;

    Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos_rel;

    // terrain related
    double terrain_pitch_angle;  // the estimated terrain angle on pitch direction

    // foot position target
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_rel; // in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target_rel; 

    bool contacts[NUM_LEG];         // flag to decide leg in the stance/swing

    // MPC output
    // TODO: do not hardcode state size, use parameter
    Eigen::Matrix<double, 18, 1> optimized_state;  //[position, euler(ZYX), foot position]
    Eigen::Matrix<double, 24, 1> optimized_input;  //[3*4 contact force,    foot velocity]

    // final results to the robot 
    Eigen::Matrix<double, NUM_DOF, 1> joint_ang_tgt;
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel_tgt;
    Eigen::Matrix<double, NUM_DOF, 1> joint_tau_tgt;


    double movement_mode = 0;
};

class LeggedJoyCmd {
    public:
    LeggedJoyCmd() {}

    // joystick command
    double velx = 0.0;
    double velx_forward = 0.0;
    double velx_backward = 0.0;
    double vely = 0.0;
    double velz = 0.0;

    double pitch_rate = 0.0;
    double roll_rate = 0.0;
    double yaw_rate = 0.0;

    double pitch_ang = 0.0;
    double roll_ang = 0.0;
    double body_height = 0.1;

    //  0 is standing, 1 is walking
    int ctrl_state = 0;
    bool ctrl_state_change_request = false;
    int prev_ctrl_state = 0;
    bool exit = false;    
};

class LeggedState {
    public:
    LeggedState() {
        ctrl.reset();
        fbk.reset();
    }

    LeggedCtrl      ctrl;
    LeggedFeedback  fbk;
    LeggedJoyCmd    joy;

    // put other unclassified variables here
};

}  // namespace legged

