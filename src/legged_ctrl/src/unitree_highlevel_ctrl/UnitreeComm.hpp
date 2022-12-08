#pragma once 
#include <Eigen/Dense>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
using namespace UNITREE_LEGGED_SDK;

#define NUM_DOF 12
#define NUM_LEG 4
class UnitreeComm {
public:
    UnitreeComm(ros::NodeHandle _nh);

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
    Eigen::Matrix<int, NUM_DOF, 1> swap_foot_indices;

private:
    ros::NodeHandle nh;

};

UnitreeComm::UnitreeComm(ros::NodeHandle _nh)
    : safe(UNITREE_LEGGED_SDK::LeggedType::Go1)
    , udp(UNITREE_LEGGED_SDK::HIGHLEVEL, 8090, "192.168.123.161", 8082)
{
    udp.InitCmdData(cmd);
    swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
    swap_foot_indices << 1, 0, 3, 2;

}