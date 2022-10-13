//
// Created by shuoy on 10/18/21.
//

#pragma once
// control time related
#define MPC_UPDATE_FREQUENCY     5.0  // ms        200Hz
#define LOW_LEVEL_CTRL_FREQUENCY 1.0  // ms       1000Hz
#define FEEDBACK_FREQUENCY       1.0  // ms       1000Hz

// constant define
// mpc
#define PLAN_HORIZON 30
#define MPC_STATE_DIM 13
#define MPC_STATE_DIM_SPARSE 12
#define MPC_CONSTRAINT_DIM 20

// robot constant
#define NUM_LEG 4
#define NUM_DOF_PER_LEG 3
#define DIM_GRF 12
#define NUM_DOF 12

#define LOWER_LEG_LENGTH 0.21

#define FOOT_FORCE_LOW 30.0
#define FOOT_FORCE_HIGH 80.0

#define FOOT_SWING_CLEARANCE1 0.0f
#define FOOT_SWING_CLEARANCE2 0.23f

#define FOOT_DELTA_X_LIMIT 0.8
#define FOOT_DELTA_Y_LIMIT 0.8

