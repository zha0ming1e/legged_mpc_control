#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


#include "LeggedState.h"

class BaseInterface {
public:
    BaseInterface(ros::NodeHandle &_nh);
    virtual ~BaseInterface() {}
    virtual bool update(double t, double dt) = 0;
    
    virtual bool send_cmd() = 0;

    LeggedState& get_legged_state() {return legged_state; }; 

    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);
    bool joy_update(double t, double dt);

    ros::NodeHandle nh;
    LeggedState legged_state;    
private:
    ros::Subscriber sub_joy_msg;

    // KF state estimator

};