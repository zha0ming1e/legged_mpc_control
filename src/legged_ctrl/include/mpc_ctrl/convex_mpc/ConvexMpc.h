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
#include "utils/LeggedContactFSM.h"
#include "mpc_ctrl/convex_mpc/ConvexQPSolver.h"


namespace legged
{
using namespace std; 
using namespace Eigen;
class ConvexMpc : public LeggedMPC
{
public:
    ConvexMpc() {}; 

    ConvexMpc(LeggedState &state);

    bool update(LeggedState &state, double t, double dt); 

    // generate target foot grf
    bool grf_update(LeggedState &state, double t, double dt);

    // generate target foot and body positions
    bool foot_update(LeggedState &state, double t, double dt);

private:

    double total_run_time;
    // leg finite state machine 
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    LeggedContactFSM leg_FSM[NUM_LEG];

    ConvexQPSolver fastConvex;
}; 


}  // namespace legged