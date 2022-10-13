#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include "LeggedState.h"
#include "estimation/BasicKF.h"
#include "wbc_ctrl/wbc.h"
#include "utils/LeggedIKSolver.h"
#include "utils/A1Kinematics.h"
#include "CasadiEKF/A1KFCombineLOWithFootTerrain.h"

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
class BaseInterface {
public:
    /* 
     * functions 
     */
    BaseInterface(ros::NodeHandle &_nh, const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile);
    virtual ~BaseInterface() {}
    virtual bool ctrl_update(double t, double dt) = 0;
    virtual bool fbk_update(double t, double dt) = 0;
    
    virtual bool send_cmd() = 0;

    LeggedState& get_legged_state() {return legged_state; }; 

    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

    // process joystick data
    bool joy_update(double t, double dt);

    // process sensor data
    bool sensor_update(double t, double dt);

    // update state estimation
    bool estimation_update(double t, double dt);

    // TODO: add a safety check, disable control or stop robot if something is wrong (large pitch angle, large roll, etc.)

    // two low level controllers
    // basic tau controller
    bool tau_ctrl_update(double t, double dt);
    // wbc controller, very important
    bool wbc_update(double t, double dt);

    /* 
     * variables 
     */
    ros::NodeHandle nh;
    LeggedState legged_state;   

    bool estimation_inited = false; 

    // pinnochio related objects 
    ModelSettings modelSettings_;
    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
    CentroidalModelInfo centroidalModelInfo_;
    std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;

    // pinocchio_state_q [position, euler, joint position]
    vector_t pinocchio_state_q;
    // pinocchio_state_v [vel, euler, joint position]
    vector_t pinocchio_state_v;

    // whole body controller 
    std::shared_ptr<Wbc> wbc_;
    int swing_leg_ctrl_type = 1;    // 1 contains WORK SPACE target 
                                    // 0 contains JOINT SPACE target
    bool verbose = false;
    // inverse kinematics solver
    std::unique_ptr<LeggedIKSolver> ik_solver;

    vector_t pos_des;
    vector_t prev_pos_des;
    vector_t vel_des;


private:
    ros::Subscriber sub_joy_msg;

    // KF state estimator
    BasicKF kf;
    // Casadi EKF state estimator
    A1SensorData ekf_data; 
    A1KFCombineLOWithFootTerrain ekf;

    // old a1 kinematics
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;
    A1Kinematics a1_kin;

    // for tau control strategy
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf_rel;        // reach ground reaction force
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_rel;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target_rel;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error_rel;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error_rel;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;       // reach a target foot location
};

}  // namespace legged
