#pragma once
// Std libraries 
#include <iostream> 
#include <string> 
#include <chrono>
#include <vector>

#include <ros/ros.h> 
#include <Eigen/Dense>

#include "LeggedParams.h"
#include "LeggedState.h"
#include "mpc_ctrl/LeggedMPC.h"
#include "utils/Utils.h"
#include "utils/MovingWindowFilter.hpp"

// Julia Libraries 
#include <julia.h>
#include "juliaCpp.h"

namespace legged
{
using namespace std; 
using namespace Eigen;
class LciMpc : public LeggedMPC
{
public:
    LciMpc(); 

    LciMpc(ros::NodeHandle &_nh); 

    bool update(LeggedState &state, double t, double dt); 

private:
    // Data members for Julia interface  
    jl_module_t* julia_mpc_module_; 
    jl_function_t* policy_function_;
    jl_function_t* update_velocity_function_; 
    jl_value_t* standing_policy_; 
    jl_value_t* walking_policy_; 
    jl_value_t* wall_climb_policy_; 
    jl_value_t* wall_walk_policy_; 
    jl_value_t* oneleg_policy_;; 
    vector<jl_value_t*> policy_args_; 
    vector<jl_value_t*> velocity_args_; 

    // add a velocity filter before sending velocities to MPC
    MovingWindowFilter pos_filter_bank[18];
    MovingWindowFilter vel_filter_bank[18];
    
}; 


}  // namespace legged