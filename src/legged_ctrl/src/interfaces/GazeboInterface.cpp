#include "interfaces/GazeboInterface.h"

GazeboInterface::GazeboInterface(ros::NodeHandle &_nh)
:BaseInterface(_nh) {

    // ROS publisher
    pub_joint_cmd[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    pub_joint_cmd[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    pub_joint_cmd[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);

    pub_joint_cmd[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    pub_joint_cmd[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    pub_joint_cmd[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);

    pub_joint_cmd[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    pub_joint_cmd[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    pub_joint_cmd[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);

    pub_joint_cmd[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    pub_joint_cmd[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    pub_joint_cmd[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);


    // ROS register callback, call backs directly modify variables in A1CtrlStates
    sub_gt_pose_msg = nh.subscribe("/torso_odom", 100, &GazeboInterface::gt_pose_callback, this);
    sub_imu_msg = nh.subscribe("/trunk_imu", 100, &GazeboInterface::imu_callback, this);

    sub_joint_msg[0] = nh.subscribe("/a1_gazebo/FL_hip_controller/state", 2, &GazeboInterface::FL_hip_state_callback, this);
    sub_joint_msg[1] = nh.subscribe("/a1_gazebo/FL_thigh_controller/state", 2, &GazeboInterface::FL_thigh_state_callback, this);
    sub_joint_msg[2] = nh.subscribe("/a1_gazebo/FL_calf_controller/state", 2, &GazeboInterface::FL_calf_state_callback, this);

    sub_joint_msg[3] = nh.subscribe("/a1_gazebo/FR_hip_controller/state", 2, &GazeboInterface::FR_hip_state_callback, this);
    sub_joint_msg[4] = nh.subscribe("/a1_gazebo/FR_thigh_controller/state", 2, &GazeboInterface::FR_thigh_state_callback, this);
    sub_joint_msg[5] = nh.subscribe("/a1_gazebo/FR_calf_controller/state", 2, &GazeboInterface::FR_calf_state_callback, this);

    sub_joint_msg[6] = nh.subscribe("/a1_gazebo/RL_hip_controller/state", 2, &GazeboInterface::RL_hip_state_callback, this);
    sub_joint_msg[7] = nh.subscribe("/a1_gazebo/RL_thigh_controller/state", 2, &GazeboInterface::RL_thigh_state_callback, this);
    sub_joint_msg[8] = nh.subscribe("/a1_gazebo/RL_calf_controller/state", 2, &GazeboInterface::RL_calf_state_callback, this);

    sub_joint_msg[9] = nh.subscribe("/a1_gazebo/RR_hip_controller/state", 2, &GazeboInterface::RR_hip_state_callback, this);
    sub_joint_msg[10] = nh.subscribe("/a1_gazebo/RR_thigh_controller/state", 2, &GazeboInterface::RR_thigh_state_callback, this);
    sub_joint_msg[11] = nh.subscribe("/a1_gazebo/RR_calf_controller/state", 2, &GazeboInterface::RR_calf_state_callback, this);

    sub_foot_contact_msg[0] = nh.subscribe("/visual/FL_foot_contact/the_force", 2, &GazeboInterface::FL_foot_contact_callback, this);
    sub_foot_contact_msg[1] = nh.subscribe("/visual/FR_foot_contact/the_force", 2, &GazeboInterface::FR_foot_contact_callback, this);
    sub_foot_contact_msg[2] = nh.subscribe("/visual/RL_foot_contact/the_force", 2, &GazeboInterface::RL_foot_contact_callback, this);
    sub_foot_contact_msg[3] = nh.subscribe("/visual/RR_foot_contact/the_force", 2, &GazeboInterface::RR_foot_contact_callback, this);

}
bool GazeboInterface::update(double t, double dt) {

    bool joy_run = joy_update(t, dt);
    // debug print some variables 
    std::cout << legged_state.joy.ctrl_state << std::endl;

    // get sensor feedback

    // update state estimator

    // run wbc 

    return joy_run;
}

bool GazeboInterface::send_cmd() {
    // send control cmd to robot via ros topic

    for (int i = 0; i < 12; i++) {
        low_cmd.motorCmd[i].mode = 0x0A;
        low_cmd.motorCmd[i].q =   legged_state.ctrl.joint_ang_tgt(i, 0);
        low_cmd.motorCmd[i].dq =  legged_state.ctrl.joint_vel_tgt(i, 0);
        low_cmd.motorCmd[i].Kp =  5;
        low_cmd.motorCmd[i].Kd =  3;
        low_cmd.motorCmd[i].tau = legged_state.ctrl.joint_tau_tgt(i, 0);
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
    }

    return true;
}


// callback functions
void GazeboInterface::gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom) {
    // update
    // legged_state.fbk.root_quat = Eigen::Quaterniond(odom->pose.pose.orientation.w,
    //                                               odom->pose.pose.orientation.x,
    //                                               odom->pose.pose.orientation.y,
    //                                               odom->pose.pose.orientation.z);                                              
    // legged_state.fbk.root_pos << odom->pose.pose.position.x,
    //         odom->pose.pose.position.y,
    //         odom->pose.pose.position.z;
    // // make sure root_lin_vel is in world frame
    // legged_state.fbk.root_lin_vel << odom->twist.twist.linear.x,
    //         odom->twist.twist.linear.y,
    //         odom->twist.twist.linear.z;

    // make sure root_ang_vel is in world frame
    // legged_state.fbk.root_ang_vel << odom->twist.twist.angular.x,
    //         odom->twist.twist.angular.y,
    //         odom->twist.twist.angular.z;



}

void GazeboInterface::imu_callback(const sensor_msgs::Imu::ConstPtr &imu) {
    legged_state.fbk.root_quat = Eigen::Quaterniond(imu->orientation.w,
                                                    imu->orientation.x,
                                                    imu->orientation.y,
                                                    imu->orientation.z);
    legged_state.fbk.imu_acc = Eigen::Vector3d(
            imu->linear_acceleration.x,
            imu->linear_acceleration.y,
            imu->linear_acceleration.z
    );
    legged_state.fbk.imu_ang_vel = Eigen::Vector3d(
            imu->angular_velocity.x,
            imu->angular_velocity.y,
            imu->angular_velocity.z
    );
}

// FL
void GazeboInterface::FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[0] = a1_joint_state.q;
    legged_state.fbk.joint_vel[0] = a1_joint_state.dq;
}

void GazeboInterface::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[1] = a1_joint_state.q;
    legged_state.fbk.joint_vel[1] = a1_joint_state.dq;
}

void GazeboInterface::FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[2] = a1_joint_state.q;
    legged_state.fbk.joint_vel[2] = a1_joint_state.dq;
}

// FR
void GazeboInterface::FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[3] = a1_joint_state.q;
    legged_state.fbk.joint_vel[3] = a1_joint_state.dq;
}

void GazeboInterface::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[4] = a1_joint_state.q;
    legged_state.fbk.joint_vel[4] = a1_joint_state.dq;
}

void GazeboInterface::FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[5] = a1_joint_state.q;
    legged_state.fbk.joint_vel[5] = a1_joint_state.dq;
}

// RL
void GazeboInterface::RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[6] = a1_joint_state.q;
    legged_state.fbk.joint_vel[6] = a1_joint_state.dq;
}

void GazeboInterface::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[7] = a1_joint_state.q;
    legged_state.fbk.joint_vel[7] = a1_joint_state.dq;
}

void GazeboInterface::RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[8] = a1_joint_state.q;
    legged_state.fbk.joint_vel[8] = a1_joint_state.dq;
}

// RR
void GazeboInterface::RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[9] = a1_joint_state.q;
    legged_state.fbk.joint_vel[9] = a1_joint_state.dq;
}

void GazeboInterface::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[10] = a1_joint_state.q;
    legged_state.fbk.joint_vel[10] = a1_joint_state.dq;
}

void GazeboInterface::RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    legged_state.fbk.joint_pos[11] = a1_joint_state.q;
    legged_state.fbk.joint_vel[11] = a1_joint_state.dq;
}

// foot contact force
void GazeboInterface::FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    legged_state.fbk.foot_force[0] = force.wrench.force.z;
}

void GazeboInterface::FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    legged_state.fbk.foot_force[1] = force.wrench.force.z;
}

void GazeboInterface::RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    legged_state.fbk.foot_force[2] = force.wrench.force.z;
}

void GazeboInterface::RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    legged_state.fbk.foot_force[3] = force.wrench.force.z;
}
