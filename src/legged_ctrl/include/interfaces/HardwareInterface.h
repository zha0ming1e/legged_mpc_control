#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>

// a1 hardware
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "LeggedState.h"
#include "interfaces/BaseInterface.h"
#include "utils/MovingWindowFilter.hpp"
#include "utils/LeggedSafetyChecker.hpp"

#define FOOT_FILTER_WINDOW_SIZE 5
namespace legged
{
using namespace ocs2;
using namespace legged_robot;

class HardwareInterface : public BaseInterface {
public:
    HardwareInterface(ros::NodeHandle &_nh, const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile);

    bool update(double t, double dt);
    
    bool send_cmd();


private:

    ros::Publisher pub_joint_angle;
    ros::Publisher pub_imu;
    sensor_msgs::JointState joint_foot_msg;
    sensor_msgs::Imu imu_msg;
    
    // a1 hardware
    UNITREE_LEGGED_SDK::UDP udp;
    UNITREE_LEGGED_SDK::Safety safe;
    UNITREE_LEGGED_SDK::LowState unitree_state = {0};
    UNITREE_LEGGED_SDK::LowCmd cmd = {0};

    void udp_init_send();

    void receive_low_state(double dt);

    // a1 hardware switch foot order
    Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
    Eigen::Matrix<int, NUM_LEG, 1> swap_foot_indices;

    // a1 hardware foot force filter
    MovingWindowFilter foot_force_filters[NUM_LEG];

    // a1 hardware joint vel filter 
    MovingWindowFilter joint_vel_filters[NUM_DOF];

    // a1 hardware safety checker
    LeggedSafetyChecker safety_checker;
};

}  // namespace legged
