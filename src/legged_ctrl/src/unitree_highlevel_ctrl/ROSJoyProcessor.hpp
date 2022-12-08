#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
// constant define
// joy stick command interprate
#define JOY_CMD_BODY_HEIGHT_MAX 0.32 // m
#define JOY_CMD_BODY_HEIGHT_MIN 0.1 // m
#define JOY_CMD_BODY_HEIGHT_VEL 0.04 // m/s
#define JOY_CMD_VELX_MAX 1.0 // m/s
#define JOY_CMD_VELY_MAX 0.5 // m/s
#define JOY_CMD_YAW_MAX 0.9 // rad
#define JOY_CMD_PITCH_MAX 0.4 // rad
#define JOY_CMD_ROLL_MAX 0.4 // rad

#define JOY_STATE_TOTAL_STATES 4

class ROSJoyProcessor {
public:
    ROSJoyProcessor(ros::NodeHandle& _nh)
    {
        nh = _nh;
        sub_joy_msg = nh.subscribe("/joy", 1000, &ROSJoyProcessor::joy_callback, this);
    }

    ~ROSJoyProcessor()
    {
        // destruct = true;
        // thread_.join();
    }

    bool isExit()
    {
        return joy_cmd_exit;
    }

    // this should be called in the control loop
    void processJoy(double dt)
    {
        if (joy_cmd_ctrl_state_change_request) {
            // toggle joy_cmd_ctrl_state
            joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
            joy_cmd_ctrl_state = joy_cmd_ctrl_state % JOY_STATE_TOTAL_STATES; // TODO: how to toggle more states?
            joy_cmd_ctrl_state_change_request = false; // erase this change request;
        }

        joy_cmd_body_height += joy_cmd_velz * dt;
        prev_joy_cmd_ctrl_state = joy_cmd_ctrl_state;
        if (joy_cmd_body_height >= JOY_CMD_BODY_HEIGHT_MAX) {
            joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX;
        }
        if (joy_cmd_body_height <= JOY_CMD_BODY_HEIGHT_MIN) {
            joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN;
        }
    }

    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        // left updown
        joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

        // A
        if (joy_msg->buttons[0] == 1) {
            joy_cmd_ctrl_state_change_request = true;
        }

        // right updown
        joy_cmd_velx = joy_msg->axes[3] * JOY_CMD_VELX_MAX;
        // right horiz
        joy_cmd_vely = joy_msg->axes[2] * JOY_CMD_VELY_MAX;
        // left horiz
        joy_cmd_yaw_rate = joy_msg->axes[0] * JOY_CMD_YAW_MAX;
        // cross button, left and right
        // joy_cmd_roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX * (-1);
        // cross button, up and down
        // joy_cmd_pitch_rate = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;

        // lb
        if (joy_msg->buttons[4] == 1) {
            std::cout << "You have pressed the exit button!!!!" << std::endl;
            joy_cmd_exit = true;
        }
    }

    // public joystick command
    double joy_cmd_velx = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_body_height = 0.28; // unitree Go1 default height
    double joy_cmd_yaw_rate = 0.0;
    //  0 is standing, 1 is walking
    int joy_cmd_ctrl_state = 0;

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_joy_msg;

    // private joystick command, intermediate
    double joy_cmd_velz = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;

    bool joy_cmd_ctrl_state_change_request = false;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_exit = false;
};