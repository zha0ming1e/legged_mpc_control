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

        fastConvex = ConvexQPSolver(state.param.q_weights, state.param.r_weights); 
    }

    bool ConvexMpc::update(LeggedState &state, double t, double dt)
    {
        total_run_time += dt;
        // foot update
        foot_update(state, t, dt);

        // grf update
        grf_update(state, t, dt);

        // now we have foot_forces_grf and FSM_foot_pos_target_world, FSM_foot_vel_target_world

        state.ctrl.root_pos_d[2] = state.joy.body_height;

        state.ctrl.optimized_state.segment<3>(0) = state.ctrl.root_pos_d;
        // reverse the euler angle direction 
        state.ctrl.optimized_state.segment<3>(3) = Eigen::Vector3d(state.ctrl.root_euler_d(2), state.ctrl.root_euler_d(1), state.ctrl.root_euler_d(0));

        for (int i = 0; i < NUM_LEG; i++)
        {
            state.ctrl.optimized_state.segment<3>(6 + 3 * i) = leg_FSM[i].FSM_foot_pos_target_world;
            state.ctrl.optimized_input.segment<3>(3 * i) = foot_forces_grf.col(i);
            state.ctrl.optimized_input.segment<3>(12 + 3 * i) = leg_FSM[i].FSM_foot_vel_target_world;
        }

        // TODO: set this flag to false if the MPC solver fails
        state.mpc_solver_inited = true;
        return true;
    }

    bool ConvexMpc::grf_update(LeggedState &state, double t, double dt)
    {
        // when entering grf update, foot contacts (plan_contacts) should be updated
        // by the foot update function and leg_FSMs

        fastConvex.calc_mpc_reference(state); 
        fastConvex.update_cons_matrix(); 
        Eigen::Matrix<double, DIM_GRF, 1> qp_solution = fastConvex.compute_grfs(state); 
        for (int i = 0; i < NUM_LEG; ++i) {
            foot_forces_grf.block<3, 1>(0, i) = state.fbk.root_rot_mat.transpose() * qp_solution.segment<3>(i * 3);
        }

        return true;
    }

    bool ConvexMpc::foot_update(LeggedState &state, double t, double dt)
    {


        for (int i = 0; i < NUM_LEG; i++)
        {
            leg_FSM[i].update(dt, state.fbk.foot_pos_world.block<3,1>(0,i), 
                                  state.ctrl.foot_pos_target_world.block<3,1>(0,i), 
                                //   state.fbk.estimated_contacts[i]);
                                  true);
        }
        // reset LegFSM in movement_mode 0
        if (state.ctrl.movement_mode == 0) {
            for (int i = 0; i < NUM_LEG; i++) leg_FSM[i].reset();
        }

        // debug 
        for (int i = 0; i < NUM_LEG; i++)
        {
            state.ctrl.plan_contacts[i] = leg_FSM[i].get_contact_state();
        }

        return true;
    }
}  // namespace legged