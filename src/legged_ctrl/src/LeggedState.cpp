/*
 *  Important shared variables between the robot interface and the cotroller 
 */

#include <Eigen/Dense>
#include <ros/ros.h>

#include "LeggedParams.h"
#include "LeggedState.h"

namespace legged
{
void LeggedFeedback::reset() {

}

void LeggedCtrl::reset() {
}

bool LeggedParam::load(ros::NodeHandle &_nh) {

    // critical parameters, if not found, return false
    if (!_nh.getParam("/run_type", run_type)) return false;
    if (!_nh.getParam("/robot_type", robot_type)) return false;
    if (!_nh.getParam("/mpc_type", mpc_type)) return false;
    if (!_nh.getParam("/low_level_type", low_level_type)) return false;

    // params that are not critical, have default values
    _nh.param("/gait_counter_speed", gait_counter_speed, 3.0); 

    double default_foot_pos_FL_x;
    double default_foot_pos_FL_y;
    double default_foot_pos_FL_z;

    double default_foot_pos_FR_x;
    double default_foot_pos_FR_y;
    double default_foot_pos_FR_z;

    double default_foot_pos_RL_x;
    double default_foot_pos_RL_y;
    double default_foot_pos_RL_z;

    double default_foot_pos_RR_x;
    double default_foot_pos_RR_y;
    double default_foot_pos_RR_z;

    _nh.param("default_foot_pos_FL_x", default_foot_pos_FL_x, 0.25);
    _nh.param("default_foot_pos_FL_y", default_foot_pos_FL_y, 0.15);
    _nh.param("default_foot_pos_FL_z", default_foot_pos_FL_z, -0.33);

    _nh.param("default_foot_pos_FR_x", default_foot_pos_FR_x, 0.25);
    _nh.param("default_foot_pos_FR_y", default_foot_pos_FR_y, -0.15);
    _nh.param("default_foot_pos_FR_z", default_foot_pos_FR_z, -0.33);

    _nh.param("default_foot_pos_RL_x", default_foot_pos_RL_x, -0.17);
    _nh.param("default_foot_pos_RL_y", default_foot_pos_RL_y, 0.15);
    _nh.param("default_foot_pos_RL_z", default_foot_pos_RL_z, -0.33);

    _nh.param("default_foot_pos_RR_x", default_foot_pos_RR_x, -0.17);
    _nh.param("default_foot_pos_RR_y", default_foot_pos_RR_y, -0.15);
    _nh.param("default_foot_pos_RR_z", default_foot_pos_RR_z, -0.33);

    default_foot_pos_rel << default_foot_pos_FL_x, default_foot_pos_FR_x, default_foot_pos_RL_x, default_foot_pos_RR_x,
            default_foot_pos_FL_y, default_foot_pos_FR_y, default_foot_pos_RL_y, default_foot_pos_RR_y,
            default_foot_pos_FL_z, default_foot_pos_FR_z, default_foot_pos_RL_z, default_foot_pos_RR_z;


    double q_weights_0, q_weights_1, q_weights_2, q_weights_3, q_weights_4, q_weights_5, q_weights_6, q_weights_7, q_weights_8, q_weights_9, q_weights_10, q_weights_11, q_weights_12, q_weights_rw_1, q_weights_rw_2;

    _nh.param("q_weights_0", q_weights_0, 80.0);
    _nh.param("q_weights_1", q_weights_1, 80.0);
    _nh.param("q_weights_2", q_weights_2, 1.0);

    _nh.param("q_weights_3", q_weights_3, 0.0);
    _nh.param("q_weights_4", q_weights_4, 0.0);
    _nh.param("q_weights_5", q_weights_5, 270.0);

    _nh.param("q_weights_6", q_weights_6, 1.0);
    _nh.param("q_weights_7", q_weights_7, 1.0);
    _nh.param("q_weights_8", q_weights_8, 20.0);

    _nh.param("q_weights_9", q_weights_9, 20.0);
    _nh.param("q_weights_10", q_weights_10, 20.0);
    _nh.param("q_weights_11", q_weights_11, 20.0);

    _nh.param("q_weights_rw_1", q_weights_rw_1, 0.0);
    _nh.param("q_weights_rw_2", q_weights_rw_2, 0.0);

    q_weights << q_weights_0, q_weights_1, q_weights_2,
            q_weights_3, q_weights_4, q_weights_5,
            q_weights_6, q_weights_7, q_weights_8,
            q_weights_9, q_weights_10, q_weights_11,
            q_weights_12;

    double r_weights_0, r_weights_1, r_weights_2, r_weights_3, r_weights_4, r_weights_5, r_weights_6, r_weights_7, r_weights_8, r_weights_9, r_weights_10, r_weights_11, r_weights_rw_1, r_weights_rw_2;

    _nh.param("r_weights_0", r_weights_0, 1e-5);
    _nh.param("r_weights_1", r_weights_1, 1e-5);
    _nh.param("r_weights_2", r_weights_2, 1e-6);

    _nh.param("r_weights_3", r_weights_3, 1e-5);
    _nh.param("r_weights_4", r_weights_4, 1e-5);
    _nh.param("r_weights_5", r_weights_5, 1e-6);

    _nh.param("r_weights_6", r_weights_6, 1e-5);
    _nh.param("r_weights_7", r_weights_7, 1e-5);
    _nh.param("r_weights_8", r_weights_8, 1e-6);

    _nh.param("r_weights_9", r_weights_9, 1e-5);
    _nh.param("r_weights_10", r_weights_10, 1e-5);
    _nh.param("r_weights_11", r_weights_11, 1e-6);

    r_weights << r_weights_0, r_weights_1, r_weights_2,
            r_weights_3, r_weights_4, r_weights_5,
            r_weights_6, r_weights_7, r_weights_8,
            r_weights_9, r_weights_10, r_weights_11;


    double kp_foot_x, kp_foot_y, kp_foot_z, kd_foot_x, kd_foot_y, kd_foot_z, km_foot_x, km_foot_y, km_foot_z;

    _nh.param("kp_foot_x", kp_foot_x, 150.0);
    _nh.param("kp_foot_y", kp_foot_y, 150.0);
    _nh.param("kp_foot_z", kp_foot_z, 200.0);

    _nh.param("kd_foot_x", kd_foot_x, 0.0);
    _nh.param("kd_foot_y", kd_foot_y, 0.0);
    _nh.param("kd_foot_z", kd_foot_z, 0.0);

    _nh.param("km_foot_x", km_foot_x, 0.1);
    _nh.param("km_foot_y", km_foot_y, 0.1);
    _nh.param("km_foot_z", km_foot_z, 0.04);

    kp_foot <<
            kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
            kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
            kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
    kd_foot <<
            kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
            kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
            kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;

    km_foot = Eigen::Vector3d(km_foot_x, km_foot_y, km_foot_z);            
    return true;
}


}  // namespace legged