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
    if (!_nh.getParam("/kf_type", kf_type)) return false;
    

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


    double q_weights_0, q_weights_1, q_weights_2, q_weights_3, q_weights_4, q_weights_5, q_weights_6, q_weights_7, q_weights_8, q_weights_9, q_weights_10, q_weights_11;

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


    q_weights.resize(12);
    r_weights.resize(12);
    q_weights << q_weights_0, q_weights_1, q_weights_2,
            q_weights_3, q_weights_4, q_weights_5,
            q_weights_6, q_weights_7, q_weights_8,
            q_weights_9, q_weights_10, q_weights_11;

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


    _nh.param("a1_robot_mass", robot_mass, 13.0);

    double a1_trunk_inertia_xx;
    double a1_trunk_inertia_xy;
    double a1_trunk_inertia_xz;
    double a1_trunk_inertia_yz;
    double a1_trunk_inertia_yy;
    double a1_trunk_inertia_zz;

    _nh.param("a1_trunk_inertia_xx", a1_trunk_inertia_xx, 0.0158533);
    _nh.param("a1_trunk_inertia_xy", a1_trunk_inertia_xy, 0.0);
    _nh.param("a1_trunk_inertia_xz", a1_trunk_inertia_xz, 0.0);
    _nh.param("a1_trunk_inertia_yz", a1_trunk_inertia_yz, 0.0);
    _nh.param("a1_trunk_inertia_yy", a1_trunk_inertia_yy, 0.0377999);
    _nh.param("a1_trunk_inertia_zz", a1_trunk_inertia_zz, 0.0456542);

    a1_trunk_inertia << a1_trunk_inertia_xx, a1_trunk_inertia_xy, a1_trunk_inertia_xz,
            a1_trunk_inertia_xy, a1_trunk_inertia_yy, a1_trunk_inertia_yz,
            a1_trunk_inertia_xz, a1_trunk_inertia_yz, a1_trunk_inertia_zz;


    // joystick mapping 
    _nh.param("/joystick_left_updown_axis", joystick_left_updown_axis, 1);     
    _nh.param("/joystick_left_horiz_axis", joystick_left_horiz_axis, 0);  
    _nh.param("/joystick_right_updown_axis", joystick_right_updown_axis, 4);  
    _nh.param("/joystick_right_horiz_axis", joystick_right_horiz_axis, 3);  
    _nh.param("/joystick_mode_switch_button", joystick_mode_switch_button, 0);  
    _nh.param("/joystick_exit_button", joystick_exit_button, 4);  

    // joystick parameters
    _nh.param("/joystick_velx_scale", joystick_velx_scale, 2.5);     
    _nh.param("/joystick_vely_scale", joystick_vely_scale, 0.4);  
    _nh.param("/joystick_height_vel", joystick_height_vel, 0.1);  
    _nh.param("/joystick_max_height", joystick_max_height, 0.3);  
    _nh.param("/joystick_min_height", joystick_min_height, 0.03);  
    _nh.param("/joystick_yaw_rate_scale", joystick_yaw_rate_scale, 0.8);  
    _nh.param("/joystick_roll_rate_scale", joystick_roll_rate_scale, 0.4);
    _nh.param("/joystick_pitch_rate_scale", joystick_pitch_rate_scale, 0.4);


    // contact detection flags
    _nh.param("/foot_sensor_max_value", foot_sensor_max_value, 300.0);     
    _nh.param("/foot_sensor_min_value", foot_sensor_min_value, 0.0);  
    _nh.param("/foot_sensor_ratio", foot_sensor_ratio, 0.5);  

    // casadi EKF parameters  
    _nh.param("/ekf_inital_cov", ekf_inital_cov, 0.001);      
    _nh.param("/ekf_noise_process_pos_xy", ekf_noise_process_pos_xy, 0.001);      
    _nh.param("/ekf_noise_process_pos_z", ekf_noise_process_pos_z, 0.001);      
    _nh.param("/ekf_noise_process_vel_xy", ekf_noise_process_vel_xy, 0.001);      
    _nh.param("/ekf_noise_process_vel_z", ekf_noise_process_vel_z, 0.01);      
    _nh.param("/ekf_noise_process_rot", ekf_noise_process_rot, 1e-6);      
    _nh.param("/ekf_noise_process_foot", ekf_noise_process_foot, 0.001);      
    _nh.param("/ekf_noise_measure_fk", ekf_noise_measure_fk, 0.01);      
    _nh.param("/ekf_noise_measure_vel", ekf_noise_measure_vel, 0.01);      
    _nh.param("/ekf_noise_measure_height", ekf_noise_measure_height, 0.0001);      
    _nh.param("/ekf_noise_opti_pos", ekf_noise_opti_pos, 0.001);      
    _nh.param("/ekf_noise_opti_vel", ekf_noise_opti_vel, 999.0);      
    _nh.param("/ekf_noise_opti_yaw", ekf_noise_opti_yaw, 0.01);      



    return true;
}


}  // namespace legged