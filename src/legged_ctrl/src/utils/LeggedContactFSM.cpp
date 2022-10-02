#include "utils/LeggedContactFSM.h"

namespace legged
{
    void LeggedContactFSM::reset_params(LeggedState &legged_state, int _leg_id)
    {
        leg_id = _leg_id;
        gait_speed = legged_state.param.gait_counter_speed;
        s = STANCE;
        set_default_gait_pattern();
    }

    void LeggedContactFSM::gait_update(double dt) {
    }

    void LeggedContactFSM::reset() {
        gait_phase = 0;
        gait_freeze = false;
        gait_freeze_counter = 0;
        gait_pattern_index = 0;
        prev_gait_pattern_index = gait_pattern_size - 1;
        cur_state_start_time = 0;
        cur_state_end_time = gait_switch_time[gait_pattern_index];
        // to handle foot hold better 
        // at the moment of reset, stance foot keep holding its position
        // swing foot move to the saved target position
        if (s == SWING) {
            FSM_foot_pos_target_abs = swing_end_foot_pos_abs;
            FSM_foot_vel_target_abs.setZero();
        }

        s = gait_state_pattern[gait_pattern_index];
        // terrain_height = 0;
        // so the outloop will keep reseting the FSM's target position
        not_first_call = false;
    }

    void LeggedContactFSM::update(double dt, Eigen::Vector3d foot_pos_cur_abs, Eigen::Vector3d  foot_pos_target_abs, bool foot_force_flag) {

        // the update is called first time in the main loop, record target position
        // in helper variables
        if (not_first_call == false) {
            swing_start_foot_pos_abs = foot_pos_cur_abs;
            swing_end_foot_pos_abs = foot_pos_target_abs;
            FSM_foot_pos_target_abs = foot_pos_target_abs;
            FSM_foot_vel_target_abs.setZero();
            not_first_call = true;
        }

        gait_phase += gait_speed * dt;
        // std::cout << gait_phase << std::endl;
        // state transition
        if (s == STANCE) {
            if (gait_phase >= cur_state_end_time) {
                stance_exit();
                swing_enter(foot_pos_cur_abs);
                s = SWING;
            }
        } else if (s == SWING) {
            if (percent_in_state() > 0.7 && foot_force_flag) {
                // if we are in the second half of the swing phase and we have early contact
                // then we should switch to stance phase immediately
                s = STANCE;
                swing_exit();
                stance_enter(foot_pos_cur_abs);
            } else if (percent_in_state()>=1.0) {
                s = STANCE;
                swing_exit();
                stance_enter(foot_pos_cur_abs);
            }
        }
        if (gait_phase > 1.0) {
            gait_phase = 0.0;
        }

        // state update 
        if (s == STANCE) {
            // do nothing
            stance_update(dt);
        } else if (s == SWING) {
            swing_update(dt, foot_pos_target_abs);
        }

    }

    // TODO: add a function to load a gaite pattern from a file
    void LeggedContactFSM::set_default_gait_pattern() {
        // default gait pattern is trotting
        gait_state_pattern.clear();
        gait_pattern_index = 0;
        gait_switch_time.clear();
        if (leg_id == 0 || leg_id == 3) {
            gait_state_pattern.push_back(STANCE);
            gait_state_pattern.push_back(SWING);
        } else {
            gait_state_pattern.push_back(SWING);
            gait_state_pattern.push_back(STANCE);
        }
        gait_switch_time.push_back(0.5);
        gait_switch_time.push_back(1.0);
        gait_pattern_size = 2;
        gait_pattern_index = 0;
        prev_gait_pattern_index = gait_pattern_size - 1;
        gait_pattern_loaded = true;
    }

    void LeggedContactFSM::set_default_stand_pattern() {
        // default gait pattern is trotting
        gait_state_pattern.clear();
        gait_pattern_index = 0;
        gait_switch_time.clear();
        gait_state_pattern.push_back(STANCE);
        gait_switch_time.push_back(1.0);
        gait_pattern_size = 1;
        gait_pattern_index = 0;
        prev_gait_pattern_index = 0;
        gait_pattern_loaded = true;
    }

    void LeggedContactFSM::common_enter() {
        // first increase pattern index
        prev_gait_pattern_index = gait_pattern_index;
        gait_pattern_index = (gait_pattern_index + 1 ) % gait_pattern_size;
        // reset cur_state_start_time,
        // notice we do not wrap gaite phase here 
        if (gait_phase > 1.0) {
            cur_state_start_time = gait_phase - 1.0;
        } else {
            cur_state_start_time = gait_phase;
        }
        cur_state_end_time = gait_switch_time[gait_pattern_index];
        std::cout << "cur_state_start_time: " << cur_state_start_time << std::endl;
        std::cout << "cur_state_end_time: " << cur_state_end_time << std::endl;
        gait_freeze = false;
        gait_freeze_counter = 0;
    }

    void LeggedContactFSM::swing_enter(Eigen::Vector3d foot_pos_cur_abs) {
        common_enter();
        swing_start_foot_pos_abs << foot_pos_cur_abs;
        swing_extend_foot_pos_abs.setZero();
    }
    void LeggedContactFSM::stance_enter(Eigen::Vector3d foot_pos_cur_abs) {
        common_enter();
        FSM_foot_pos_target_abs << foot_pos_cur_abs;
        FSM_foot_vel_target_abs.setZero();

        // record the height when we enter stance phase
        terrain_height = foot_pos_cur_abs[2];
    }

    void LeggedContactFSM::swing_update(double dt, Eigen::Vector3d foot_pos_target_abs) {
        // generate swing trajectory
        double t = percent_in_state();
        Eigen::Matrix<double, 6,1> foot_pos_vel_target = 
            bezierUtils.get_foot_pos_curve(t,
                    swing_start_foot_pos_abs,
                    foot_pos_target_abs+swing_extend_foot_pos_abs, 0.0);

        swing_end_foot_pos_abs = foot_pos_target_abs;       
        FSM_prev_foot_pos_target_abs = FSM_foot_pos_target_abs;             
        FSM_foot_pos_target_abs = foot_pos_vel_target.segment(0,3);
        FSM_foot_vel_target_abs = (FSM_foot_pos_target_abs - FSM_prev_foot_pos_target_abs)/dt;
    }

    double LeggedContactFSM::percent_in_state() {
        double percent = (gait_phase-cur_state_start_time)/(cur_state_end_time-cur_state_start_time);
        if (percent < 0.0) {
            percent = 0.0;
        } else if (percent > 1.0) {
            percent = 1.0;
        }

        return percent;
    }

} // namespace legged
