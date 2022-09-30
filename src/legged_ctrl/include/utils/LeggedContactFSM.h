#pragma once
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "LeggedParams.h"
#include "LeggedState.h"
#include "utils/Utils.h"

namespace legged
{
    enum LeggedContactState
    {
        STANCE,
        SWING
    };

    class LeggedContactFSM
    {
        public:
            LeggedContactFSM() {set_default_gait_pattern();};
            void reset_params(LeggedState &legged_state, int _leg_id);
            void set_default_gait_pattern();

            LeggedContactState get_contact_state() {return state_;}

            Eigen::Vector3d get_pos_target() {return FSM_foot_pos_target_world;}
            Eigen::Vector3d get_vel_target() {return FSM_foot_vel_target_world;}

            // foot_pos_cur_world is the feedback foot position, in stance phase we should hold this position
            // foot_pos_target_world is the raibert strategy target position, in swing phase we should move to this position
            void update(double dt, Eigen::Vector3d &foot_pos_cur_world, Eigen::Vector3d & foot_pos_target_world, double foot_force);

            // reset everything to the start of the gait pattern
            void reset();

            void swing_enter(Eigen::Vector3d &foot_pos_cur_world);
            void swing_update(double dt);
            void swing_exit() {};

            // enter function that both swing and stance can use
            void common_enter();

            void stance_enter(Eigen::Vector3d &foot_pos_cur_world);
            void stance_update(double dt);
            void stance_exit() {};

            double percent_in_state() {
                return (gait_phase-cur_state_start_time)/(cur_state_end_time-cur_state_start_time);
            }

        private:
            int leg_id;
            // leg finite state machine
            LeggedContactState s;
            double gait_phase;   // 0-1
            double gait_speed;
            // gait pattern definition
            std::vector<LeggedContactState> gait_state_pattern;
            std::vector<double> gait_switch_time;
            int gait_pattern_size;
            bool gait_pattern_loaded = false;
            // gait execution control
            int gait_pattern_index;
            int prev_gait_pattern_index;
            // record time within one state
            double cur_state_start_time; // 0-1 phase time 
            double cur_state_end_time;   // 0-1 phase time 

            // either swing or stance, the getTarget function will return the target foot position
            Eigen::Vector3d FSM_foot_pos_target_world;
            Eigen::Vector3d FSM_foot_vel_target_world;

            // swing phase helper variables
            Eigen::Vector3d swing_start_foot_pos_world;
            Eigen::Vector3d swing_end_foot_vel_world;
            Eigen::Vector3d swing_cur_foot_vel_world;


    };
}  // namespace legged