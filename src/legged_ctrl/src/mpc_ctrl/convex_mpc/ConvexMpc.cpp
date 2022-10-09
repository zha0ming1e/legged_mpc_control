#include "mpc_ctrl/convex_mpc/ConvexMpc.h"
#include "utils/Utils.h"
#include "utils/MovingWindowFilter.hpp"
namespace legged
{
    ConvexMpc::ConvexMpc(LeggedState &state)
    {
        for (int i = 0; i < NUM_LEG; i++)
        {
            leg_FSM[i].reset_params(state, i);
        }
        total_run_time = 0;

        // notice we scale weights by control dt
        fastConvex = ConvexQPSolver(state.param.q_weights, 
                                    state.param.r_weights); 

        root_lin_vel_d_rel_filter_x = MovingWindowFilter(200);
        root_lin_vel_d_rel_filter_y = MovingWindowFilter(200);

    }

    bool ConvexMpc::update(LeggedState &state, double t, double dt)
    {
        total_run_time += dt;
        if (state.estimation_inited == false)
        {
            std::cout << "estimation not inited" << std::endl;
            return true;
        }
        // read joy command 
        state.ctrl.root_pos_d[2] = state.joy.body_height;
        // TODO: add a filter here for lin_vel_d_rel
        state.ctrl.root_lin_vel_d_rel[0] = root_lin_vel_d_rel_filter_x.CalculateAverage(state.joy.velx);
        state.ctrl.root_lin_vel_d_rel[1] = root_lin_vel_d_rel_filter_y.CalculateAverage(state.joy.vely);
        state.ctrl.root_ang_vel_d_rel[2] = state.joy.yaw_rate;
        state.ctrl.root_euler_d[2] += state.joy.yaw_rate * dt;

        // foot update 
        // TODO: this function can totally be moved to the low level control thread to make the raibert strategy more responsive
        foot_update(state, t, dt);

        // grf update
        grf_update(state, t, dt);

        // now we have foot_forces_grf_world and FSM_foot_pos_target_abs, FSM_foot_vel_target_abs
        // assemble them into optimized_state and optimized_input so low level controller can use them
        state.ctrl.optimized_state.segment<3>(0) = state.ctrl.root_pos_d;
        state.ctrl.optimized_state.segment<3>(3) = state.ctrl.root_euler_d;

        for (int i = 0; i < NUM_LEG; i++)
        {
            state.ctrl.optimized_input.segment<3>(3 * i) = foot_forces_grf_world.col(i);
            state.ctrl.optimized_state.segment<3>(6 + 3 * i) =  leg_FSM[i].FSM_foot_pos_target_world;
            state.ctrl.optimized_input.segment<3>(12 + 3 * i) =  leg_FSM[i].FSM_foot_vel_target_world;
        }

        // TODO: set this flag to false if the MPC solver fails
        state.mpc_solver_inited = true;
        return true;
    }

    bool ConvexMpc::grf_update(LeggedState &state, double t, double dt)
    {
        // when entering grf update, foot contacts (plan_contacts) should be updated
        // by the foot update function and leg_FSMs

        // TODO: pass legFSM into the convex MPC solver to predict contact 
        fastConvex.calc_mpc_reference(state, leg_FSM); 
        fastConvex.update_cons_matrix(); 
        Eigen::Matrix<double, DIM_GRF, 1> qp_solution = fastConvex.compute_grfs(state); 
        for (int i = 0; i < NUM_LEG; ++i) {
            foot_forces_grf_world.block<3, 1>(0, i) = qp_solution.segment<3>(i * 3);
        }
        // std::cout << foot_forces_grf_world << std::endl;
        return true;
    }

    bool ConvexMpc::foot_update(LeggedState &state, double t, double dt)
    {   
        // in low level loop, a raibert strategy generates foot_pos_target_world
        // stored in state.ctrl.foot_pos_target_world

        // reset LegFSM in movement_mode 0
        if (state.ctrl.movement_mode == 0) {
            for (int i = 0; i < NUM_LEG; i++) {
                leg_FSM[i].reset();
                // movement_mode 0: only do torque control so foot positions are not important
                // we only reset leg FSM for safety
                state.ctrl.plan_contacts[i] = true;
            }
        } else {
            for (int i = 0; i < NUM_LEG; i++) {
                leg_FSM[i].update(dt, state.fbk.foot_pos_world.block<3,1>(0,i), 
                                    state.ctrl.foot_pos_target_world.block<3,1>(0,i), 
                                    state.fbk.estimated_contacts[i]);

                // TODO: gait phase of each leg is individually controlled, so we may need to occasionally synchrinize them, for example if all leg are in contact, average their gait phase and set them to that value
            }        
            for (int i = 0; i < NUM_LEG; i++)
            {
                state.ctrl.plan_contacts[i] = leg_FSM[i].get_contact_state();
            }
        }

        return true;
    }
}  // namespace legged