//
// Created by shuoy on 10/19/21.
//

#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "LeggedParams.h"

class Utils {
public:
    // compare to Eigen's default eulerAngles
    // this function returns yaw angle within -pi to pi
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);
    // lqy's ZYX order 
    static Eigen::Vector3d quatToZyx(const Eigen::Quaterniond q);
    static double square(double a) { return a * a; }
    static Eigen::Vector3d quat_to_rotVec(Eigen::Quaterniond quat);
    static Eigen::Matrix3d skew(Eigen::Vector3d vec);
    static Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat);
    static double cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2);

    // pinnochio joint order: FL_shoulder FL_hip FL_knee RL_shoulder RL_hip RL_knee FR_shoulder FR_hip FR_knee  RR_shoulder RR_hip RR_knee
    // unitree   joint order: FL_shoulder FL_hip FL_knee FR_shoulder FR_hip FR_knee RL_shoulder RL_hip RL_knee  RR_shoulder RR_hip RR_knee
    static Eigen::Matrix<double, NUM_DOF,1> joint_vec_unitree_to_pinnochio(Eigen::Matrix<double, NUM_DOF,1> vec);
    static Eigen::Matrix<double, NUM_DOF,1> joint_vec_pinnochio_to_unitree(Eigen::Matrix<double, NUM_DOF,1> vec);
};

class BezierUtils {
    // https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html
public:
    BezierUtils () {
        curve_constructed = false;
        bezier_degree = 4;
    }
    // set of functions create bezier curves, get points, reset
    Eigen::Matrix<double, 6,1> get_foot_pos_curve(float t,
                                       Eigen::Vector3d foot_pos_start,
                                       Eigen::Vector3d foot_pos_final,
                                       double terrain_pitch_angle);

    void reset_foot_pos_curve() {curve_constructed = false;}
private:
    Eigen::Vector2d bezier_curve(double t, const std::vector<double> &P);    

    double binomial(int n, int k);
    double bezier_coefficient(double t, int n, int k);

    bool curve_constructed;
    float bezier_degree;
};


