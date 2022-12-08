#include <iostream>
#include <math.h>
#include <string.h>
#include <unistd.h>

#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

#include "ROSJoyProcessor.hpp"
#include "UnitreeComm.hpp"

#define FOOT_FILTER_WINDOW_SIZE 20

// read joystick data
int main(int argc, char** argv)
{
    ros::init(argc, argv, "unitree_highlevel_ctrl");

    ros::NodeHandle nh;

    // std::cout << "Communication level is set to HIGH-level." << std::endl
    //           << "WARNING: Make sure the robot is standing on the ground." << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();

    UnitreeComm unitreeComm(nh);
    ROSJoyProcessor rosJoy(nh);

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/hardware_go1/imu", 100);
    ros::Publisher joint_foot_pub = nh.advertise<sensor_msgs::JointState>("/hardware_go1/joint_foot", 100);
    ros::Publisher estimated_odom_pub = nh.advertise<nav_msgs::Odometry>("/hardware_go1/estimated_odom", 100);

    ros::Rate loop_rate(100);

    // foot force filter
    Eigen::Matrix<double, NUM_LEG, FOOT_FILTER_WINDOW_SIZE> foot_force_filters;
    Eigen::Matrix<int, NUM_LEG, 1> foot_force_filters_idx;
    Eigen::Matrix<double, NUM_LEG, 1> foot_force_filters_sum;
    Eigen::Matrix<double, NUM_LEG, 1> init_force_bias;

    foot_force_filters.setZero();
    foot_force_filters_idx.setZero();
    foot_force_filters_sum.setZero();

    bool bias_recorded = false;
    bool use_foot_force_filter = true;

    int loop_count = 0;
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);

    // unitree cmd setup
    unitreeComm.cmd.mode = 0; // 0:idle, default stand      1:forced stand     2:walk continuously
    unitreeComm.cmd.gaitType = 0;
    unitreeComm.cmd.speedLevel = 0;
    unitreeComm.cmd.footRaiseHeight = 0;
    unitreeComm.cmd.bodyHeight = 0;
    unitreeComm.cmd.euler[0] = 0;
    unitreeComm.cmd.euler[1] = 0;
    unitreeComm.cmd.euler[2] = 0;
    unitreeComm.cmd.velocity[0] = 0.0f;
    unitreeComm.cmd.velocity[1] = 0.0f;
    unitreeComm.cmd.yawSpeed = 0.0f;
    unitreeComm.cmd.reserve = 0;

    // rosJoy exits then the controller shuts down
    while (ros::ok() && rosJoy.isExit() == false) {
        // get time
        now = ros::Time::now();
        dt = now - prev;
        prev = now;
        double dt_s = dt.toSec();

        // joy update
        rosJoy.processJoy(dt_s);

        // receive state through udp
        unitreeComm.udp.Recv();
        unitreeComm.udp.GetRecv(unitreeComm.state);

        // unitree estimation
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.pose.pose.position.x = unitreeComm.state.position[0];
        odom.pose.pose.position.y = unitreeComm.state.position[1];
        odom.pose.pose.position.z = unitreeComm.state.position[2];
        odom.twist.twist.linear.x = unitreeComm.state.velocity[0];
        odom.twist.twist.linear.y = unitreeComm.state.velocity[1];
        odom.twist.twist.linear.z = unitreeComm.state.velocity[2];

        estimated_odom_pub.publish(odom);

        // imu message
        sensor_msgs::Imu imu_msg;

        imu_msg.header.stamp = now;

        imu_msg.linear_acceleration.x = unitreeComm.state.imu.accelerometer[0];
        imu_msg.linear_acceleration.y = unitreeComm.state.imu.accelerometer[1];
        imu_msg.linear_acceleration.z = unitreeComm.state.imu.accelerometer[2];

        imu_msg.angular_velocity.x = unitreeComm.state.imu.gyroscope[0];
        imu_msg.angular_velocity.y = unitreeComm.state.imu.gyroscope[1];
        imu_msg.angular_velocity.z = unitreeComm.state.imu.gyroscope[2];

        imu_pub.publish(imu_msg);

        // joint state message
        sensor_msgs::JointState joint_foot_msg;

        joint_foot_msg.header.stamp = now;

        joint_foot_msg.name = { "FL0", "FL1", "FL2",
            "FR0", "FR1", "FR2",
            "RL0", "RL1", "RL2",
            "RR0", "RR1", "RR2",
            "FL_foot", "FR_foot", "RL_foot", "RR_foot" };

        joint_foot_msg.position.resize(NUM_DOF + NUM_LEG);
        joint_foot_msg.velocity.resize(NUM_DOF + NUM_LEG);
        joint_foot_msg.effort.resize(NUM_DOF + NUM_LEG);

        for (int i = 0; i < NUM_DOF; i++) {
            int swap_i = unitreeComm.swap_joint_indices(i);
            joint_foot_msg.position[i] = unitreeComm.state.motorState[swap_i].q;
            joint_foot_msg.velocity[i] = unitreeComm.state.motorState[swap_i].dq;
        }

        // read foot_force
        if (use_foot_force_filter == true) {
            if (bias_recorded == false && loop_count >= 1000) {
                for (int i = 0; i < NUM_LEG; ++i) {
                    int swap_i = unitreeComm.swap_foot_indices(i);
                    init_force_bias[i] = unitreeComm.state.footForce[swap_i];
                }
                bias_recorded = true;
            }

            for (int i = 0; i < NUM_LEG; ++i) {
                int swap_i = unitreeComm.swap_foot_indices(i);
                double value = static_cast<double>(unitreeComm.state.footForce[swap_i]) - init_force_bias[i];

                foot_force_filters_sum[i] -= foot_force_filters(i, foot_force_filters_idx[i]);
                foot_force_filters(i, foot_force_filters_idx[i]) = value;
                foot_force_filters_sum[i] += value;
                foot_force_filters_idx[i]++;
                foot_force_filters_idx[i] %= FOOT_FILTER_WINDOW_SIZE;

                joint_foot_msg.effort[NUM_DOF + i] = foot_force_filters_sum[i] / static_cast<double>(FOOT_FILTER_WINDOW_SIZE);
            }

        } else {
            for (int i = 0; i < NUM_LEG; i++) {
                int swap_i = unitreeComm.swap_foot_indices(i);
                joint_foot_msg.effort[NUM_DOF + i] = unitreeComm.state.footForce[swap_i];
            }
        }

        joint_foot_pub.publish(joint_foot_msg);

        // test joy read
        // std::cout << "Joystick cmd: " << rosJoy.joy_cmd_ctrl_state << " - " << rosJoy.joy_cmd_velx << " - " << rosJoy.joy_cmd_vely << std::endl;

        // update cmd using rosJoy variables
        unitreeComm.cmd.mode = 2;
        unitreeComm.cmd.gaitType = rosJoy.joy_cmd_ctrl_state;
        // unitreeComm.cmd.gaitType = 3;
        unitreeComm.cmd.velocity[0] = rosJoy.joy_cmd_velx;
        unitreeComm.cmd.velocity[1] = rosJoy.joy_cmd_vely;
        unitreeComm.cmd.velocity[2] = rosJoy.joy_cmd_yaw_rate;

        unitreeComm.udp.SetSend(unitreeComm.cmd);
        unitreeComm.udp.Send();

        ros::spinOnce();

        loop_rate.sleep();
        loop_count++;
    };

    return 0;
}