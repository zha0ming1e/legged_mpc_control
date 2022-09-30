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


namespace legged
{
using namespace std; 
using namespace Eigen;
class ConvexMpc : public LeggedMPC
{
public:
    ConvexMpc(); 

    ConvexMpc(ros::NodeHandle &_nh); 

    bool update(LeggedState &state, double t, double dt); 

    // generate target foot grf
    bool grf_udate(LeggedState &state, double t, double dt);

    // generate target foot and body positions
    bool foot_udate(LeggedState &state, double t, double dt);

private:

    double total_run_time;
    // leg finite state machine 
}; 


}  // namespace legged