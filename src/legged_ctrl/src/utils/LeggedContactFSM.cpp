#include "utils/LeggedContactFSM.h"

namespace legged
{
    void LeggedContactFSM::reset_params(LeggedState &legged_state, int _leg_id)
    {
        leg_id = _leg_id;
        gait_speed = legged_state.param.gait_counter_speed;
        s = STANCE;
    }
    void LeggedContactFSM::update(double dt, Eigen::Vector3d &foot_pos_cur_world, Eigen::Vector3d & foot_pos_target_world, double foot_force) {
        gait_phase += gait_speed * dt;
        if (gait_phase > 1.0) {
            gait_phase -= 1.0;
        }
        // state transition
        if (s == STANCE) {
            if (foot_force > 0.1) {
                s = SWING;
                swing_enter();
            }
        } else if (s == SWING) {
            if (foot_force < 0.1) {
                s = STANCE;
                stance_enter();
            }
        }

        // state update 
        if (s == STANCE) {
            stance_update(dt);
        } else if (s == SWING) {
            swing_update(dt);
        }
    }

    // TODO: add a function to load a gaite pattern from a file
    void LeggedContactFSM::set_default_gait_pattern() {
        // default gait pattern is trotting
        gait_state_pattern.clear();
        gait_pattern_index = 0;
        if (leg_id % 2 == 0) {
            gait_state_pattern.push_back(STANCE);
            gait_state_pattern.push_back(SWING);
        } else {
            gait_state_pattern.push_back(SWING);
            gait_state_pattern.push_back(STANCE);
        }
        gait_switch_time.clear();
        gait_switch_time.push_back(0.5);
        gait_switch_time.push_back(1.0);
        gait_pattern_size = 2;
        gait_pattern_index = 0;
        prev_gait_pattern_index = gait_pattern_size - 1;
        gait_pattern_loaded = true;
    }

    void LeggedContactFSM::common_enter() {
        // first increase pattern index
        prev_gait_pattern_index = gait_pattern_index;
        gait_pattern_index = (gait_pattern_index + 1 ) % gait_pattern_size;
        // reset time within one state
        if (cur_state_end_time < 1.0) {
            cur_state_start_time = cur_state_end_time;
        } else {
            cur_state_start_time = 0.0;
        }
        cur_state_end_time = gait_switch_time[gait_pattern_index];

    }

    void LeggedContactFSM::swing_enter(Eigen::Vector3d &foot_pos_cur_world) {
        common_enter();
        swing_start_foot_pos_world << foot_pos_cur_world;
    }
    void LeggedContactFSM::stance_enter(Eigen::Vector3d &foot_pos_cur_world) {
        common_enter();
        FSM_foot_pos_target_world << foot_pos_cur_world;
        FSM_foot_pos_target_world.setZero();
    }

} // namespace legged
