//
//  Created by chiyenl on 05/5/22
//

#pragma once
#include <iostream>
#include <string>
#include <chrono> 
#include <unordered_map> 

// Ros publisher
#include <ros/console.h> 
#include <ros/ros.h> 
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h> 
#include <sensor_msgs/JointState.h> 

#include "LeggedParams.h"
#include "LeggedState.h"

using namespace std; 

namespace legged
{
class LeggedLogger {

public: 
    LeggedLogger(); 
    
    LeggedLogger(ros::NodeHandle &_nh) {
        nh_ = _nh; 

        // Initialize foot debug publisher
        string prefix = "/a1_debug";
        for(size_t i = 0; i < NUM_LEG; ++i) {
            string topic_name = prefix + foot_name_map_[i] +  "/pose"; 
            string topic_name_target = prefix + foot_name_map_[i] + "/target";

            pub_foot_pose_[i] = nh_.advertise<visualization_msgs::Marker>(topic_name, 100);
            pub_foot_pose_target_[i] = nh_.advertise<visualization_msgs::Marker>(topic_name_target, 100); 

            // initialize marker info
            foot_marker_[i].header.frame_id = "a1_world";
            foot_marker_[i].ns = "basic_shapes";
            foot_marker_[i].id = 10 + i;
            foot_marker_[i].type = visualization_msgs::Marker::CYLINDER;
            foot_marker_[i].action = visualization_msgs::Marker::ADD;

            foot_marker_[i].scale.x = 0.08;
            foot_marker_[i].scale.y = 0.08;
            foot_marker_[i].scale.z = 0.02;
            foot_marker_[i].pose.orientation.x = 0.0;
            foot_marker_[i].pose.orientation.y = 0.0;
            foot_marker_[i].pose.orientation.z = 0.0;
            foot_marker_[i].pose.orientation.w = 1.0;

            foot_marker_[i].color.r = 1.0f;
            foot_marker_[i].color.g = 0.0f;
            foot_marker_[i].color.b = 0.0f;
            foot_marker_[i].color.a = 1.0;
        }

        // Initialize robot state publisher
        pub_root_pose_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/odom", 100); 
        pub_euler_ = nh_.advertise<geometry_msgs::Vector3Stamped>(prefix + "/euler", 100); 
        pub_joint_data_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/joint", 100); 
        joint_data_.name = {"FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf"};
        joint_data_.effort.resize(12); 
        joint_data_.position.resize(12); 
        joint_data_.velocity.resize(12); 

        // Initialize desired state publisher
        pub_root_pose_d_ = nh_.advertise<nav_msgs::Odometry>(prefix + "/odom_d", 100); 
        pub_euler_d_ = nh_.advertise<geometry_msgs::Vector3Stamped>(prefix + "/euler_d", 100); 

        // Initialize ground reaction forces
        pub_grf_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/stance_foot_forces", 100); 
        pub_swing_forces_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/swing_foot_forces", 100); 
        grf_msg_.name = {"fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR"}; 
        swing_forces_msg_.name = {"fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR"}; 
        grf_msg_.effort.resize(12);
        swing_forces_msg_.effort.resize(12); 

        // Initialize contact forces 
        pub_contact_forces_ = nh_.advertise<sensor_msgs::JointState>(prefix + "/contact_forces", 100); 
        contact_forces_msg_.name = {"FL, FR, RL, RR"}; 
        contact_forces_msg_.effort.resize(4); 


    }

    void publish_state(LeggedState &state, double t) {
        // Fill out the robot pose 
        auto stamp_now = ros::Time::now(); 
        odom_.header.stamp = stamp_now;
        odom_.pose.pose.position.x = state.fbk.root_pos(0);
        odom_.pose.pose.position.y = state.fbk.root_pos(1); 
        odom_.pose.pose.position.z = state.fbk.root_pos(2); 

        odom_.pose.pose.orientation.w = state.fbk.root_quat.w(); 
        odom_.pose.pose.orientation.x = state.fbk.root_quat.x(); 
        odom_.pose.pose.orientation.y = state.fbk.root_quat.y(); 
        odom_.pose.pose.orientation.z = state.fbk.root_quat.z(); 

        odom_.twist.twist.linear.x = state.fbk.root_lin_vel(0);
        odom_.twist.twist.linear.y = state.fbk.root_lin_vel(1);
        odom_.twist.twist.linear.z = state.fbk.root_lin_vel(2); 

        odom_.twist.twist.angular.x = state.fbk.root_ang_vel(0); 
        odom_.twist.twist.angular.y = state.fbk.root_ang_vel(1); 
        odom_.twist.twist.angular.z = state.fbk.root_ang_vel(2); 

        euler_.vector.x = state.fbk.root_euler(0); 
        euler_.vector.y = state.fbk.root_euler(1); 
        euler_.vector.z = state.fbk.root_euler(2); 

        // fill out desired pose
        odom_d_.header.stamp = stamp_now; 
        odom_d_.pose.pose.position.x = state.ctrl.root_pos_d(0);
        odom_d_.pose.pose.position.y = state.ctrl.root_pos_d(1); 
        odom_d_.pose.pose.position.z = state.ctrl.root_pos_d(2); 

        odom_d_.twist.twist.linear.x = state.ctrl.root_lin_vel_d_rel(0);
        odom_d_.twist.twist.linear.y = state.ctrl.root_lin_vel_d_rel(1);
        odom_d_.twist.twist.linear.z = state.ctrl.root_lin_vel_d_rel(2); 

        odom_d_.twist.twist.angular.x = state.ctrl.root_ang_vel_d_rel(0); 
        odom_d_.twist.twist.angular.y = state.ctrl.root_ang_vel_d_rel(1); 
        odom_d_.twist.twist.angular.z = state.ctrl.root_ang_vel_d_rel(2); 

        euler_d_.header.stamp = stamp_now;
        euler_d_.vector.x = state.ctrl.root_euler_d(0); 
        euler_d_.vector.y = state.ctrl.root_euler_d(1); 
        euler_d_.vector.z = state.ctrl.root_euler_d(2); 

        // Grfs and joints data 
        joint_data_.header.stamp = stamp_now; 
        grf_msg_.header.stamp = stamp_now; 
        contact_forces_msg_.header.stamp = stamp_now; 
        swing_forces_msg_.header.stamp = stamp_now; 
        for(size_t i = 0; i < NUM_LEG; ++i) {
            foot_marker_[i].header.stamp = stamp_now; 
            foot_marker_[i].pose.position.x = state.fbk.foot_pos_rel(0, i); 
            foot_marker_[i].pose.position.y = state.fbk.foot_pos_rel(1, i); 
            foot_marker_[i].pose.position.z = state.fbk.foot_pos_rel(2, i); 

            foot_marker_target_[i].pose.position.x = state.ctrl.foot_pos_target_rel(0,i); 
            foot_marker_target_[i].pose.position.y = state.ctrl.foot_pos_target_rel(1,i);
            foot_marker_target_[i].pose.position.z = state.ctrl.foot_pos_target_rel(2,i);
            
            joint_data_.position[i*3] = state.fbk.joint_pos[i*3]; 
            joint_data_.position[i*3+1] = state.fbk.joint_pos[i*3+1]; 
            joint_data_.position[i*3+2] = state.fbk.joint_pos[i*3+2]; 

            joint_data_.effort[i*3] = state.ctrl.joint_tau_tgt[i*3]; 
            joint_data_.effort[i*3+1] = state.ctrl.joint_tau_tgt[i*3+1];
            joint_data_.effort[i*3+2] = state.ctrl.joint_tau_tgt[i*3+2]; 

            joint_data_.velocity[i*3] = state.fbk.joint_vel[i*3]; 
            joint_data_.velocity[i*3+1] = state.fbk.joint_vel[i*3+1]; 
            joint_data_.velocity[i*3+2] = state.fbk.joint_vel[i*3+2]; 

            // grf_msg_.effort[i*3] = state.fbk.foot_forces_grf(0, i); 
            // grf_msg_.effort[i*3+1] = state.fbk.foot_forces_grf(1, i); 
            // grf_msg_.effort[i*3+2] = state.fbk.foot_forces_grf(2, i); 

            // swing_forces_msg_.effort[i*3] = state.fbk.foot_forces_kin(0, i); 
            // swing_forces_msg_.effort[i*3+1] = state.fbk.foot_forces_kin(1, i); 
            // swing_forces_msg_.effort[i*3+2] = state.fbk.foot_forces_kin(2, i); 

            pub_foot_pose_[i].publish(foot_marker_[i]); 
            pub_foot_pose_target_[i].publish(foot_marker_target_[i]); 

            contact_forces_msg_.effort[i] = state.fbk.foot_force[i]; 
        }

        // publish 
        pub_root_pose_.publish(odom_); 
        pub_root_pose_d_.publish(odom_d_); 
        pub_euler_.publish(euler_); 
        pub_euler_d_.publish(euler_d_); 
        pub_grf_.publish(grf_msg_); 
        pub_joint_data_.publish(joint_data_); 
        pub_contact_forces_.publish(contact_forces_msg_); 
        pub_swing_forces_.publish(swing_forces_msg_); 
    }

private: 
    ros::NodeHandle nh_;

    // Publish robot state
    ros::Publisher pub_root_pose_;   // rigidbody state 
    ros::Publisher pub_euler_;       // extra publisher for euler angles 
    ros::Publisher pub_joint_data_;  // joint data
    nav_msgs::Odometry odom_;        
    geometry_msgs::Vector3Stamped euler_;
    sensor_msgs::JointState joint_data_;  
    
    // Publish desired root state 
    ros::Publisher pub_root_pose_d_; // desired rigidbody state 
    ros::Publisher pub_euler_d_;     // desired angle pose 
    nav_msgs::Odometry odom_d_;      
    geometry_msgs::Vector3Stamped euler_d_; 

    // Publish ground reaction forces 
    ros::Publisher pub_grf_;           // ground reactions forces 
    ros::Publisher pub_swing_forces_;  // swing forces 
    sensor_msgs::JointState grf_msg_;
    sensor_msgs::JointState swing_forces_msg_; 

    // Robot foot pose and target food pose 
    ros::Publisher pub_foot_pose_[NUM_LEG];
    ros::Publisher pub_foot_pose_target_[NUM_LEG]; 
    visualization_msgs::Marker foot_marker_[NUM_LEG];
    visualization_msgs::Marker foot_marker_target_[NUM_LEG]; 
    
    // publish contact info 
    ros::Publisher pub_contact_forces_; 
    sensor_msgs::JointState contact_forces_msg_; 

    // Foot name mapping 
    unordered_map<int, string> foot_name_map_{{0, "/FL"},
                                              {1, "/FR"},
                                              {2, "/RL"},
                                              {3, "/RR"}}; 


};

} // namespace legged