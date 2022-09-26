
#include "interfaces/BaseInterface.h"

BaseInterface::BaseInterface(ros::NodeHandle &_nh) {
    nh = _nh;

    sub_joy_msg = nh.subscribe("/joy", 1000, &BaseInterface::joy_callback, this);
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