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
        state.ctrl.root_lin_vel_d_rel[0] = state.joy.velx;
        state.ctrl.root_lin_vel_d_rel[1] = state.joy.vely;
        state.ctrl.root_ang_vel_d_rel[2] = state.joy.yaw_rate;
        // set default foot position to be on the ground according to target body height
        state.param.default_foot_pos_rel.row(2) = -(state.joy.body_height-0.03) * Eigen::VectorXd::Ones(NUM_LEG);

        // foot update
        foot_update(state, t, dt);

        // grf update
        grf_update(state, t, dt);

        // now we have foot_forces_grf and FSM_foot_pos_target_world, FSM_foot_vel_target_world

        state.ctrl.optimized_state.segment<3>(0) = state.ctrl.root_pos_d;
        // reverse the euler angle direction 
        // state.ctrl.optimized_state.segment<3>(3) = Eigen::Vector3d(state.ctrl.root_euler_d(2), state.ctrl.root_euler_d(1), state.ctrl.root_euler_d(0));
        state.ctrl.optimized_state.segment<3>(3) = Eigen::Vector3d::Zero();

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
        std::cout << foot_forces_grf << std::endl;
        return true;
    }

    bool ConvexMpc::foot_update(LeggedState &state, double t, double dt)
    {
        // always calculate Raibert Heuristic, calculate foothold position
        // update foot plan: state.foot_pos_target_world
        Eigen::Vector3d lin_vel_world = state.fbk.root_lin_vel; // world frame linear velocity
        Eigen::Vector3d lin_vel_rel = state.fbk.root_rot_mat_z.transpose() * lin_vel_world; // robot body frame linear velocity

        state.ctrl.foot_pos_target_rel = state.param.default_foot_pos_rel;
        for (int i = 0; i < NUM_LEG; ++i) {
            double delta_x =
                    std::sqrt(std::abs(state.param.default_foot_pos_rel(2)) / 9.8) * (lin_vel_rel(0) - state.ctrl.root_lin_vel_d_rel(0)) +
                    ((0.5*1/state.param.gait_counter_speed)  * dt) / 2.0 *
                    state.ctrl.root_lin_vel_d_rel(1);
                    state.ctrl.root_lin_vel_d_rel(0);
            double delta_y =
                    std::sqrt(std::abs(state.param.default_foot_pos_rel(2)) / 9.8) * (lin_vel_rel(1) - state.ctrl.root_lin_vel_d_rel(1)) +
                    ((0.5*1/state.param.gait_counter_speed) * dt) / 2.0 *
                    state.ctrl.root_lin_vel_d_rel(1);
                    state.ctrl.root_lin_vel_d_rel(1);

            if (delta_x < -FOOT_DELTA_X_LIMIT) {
                delta_x = -FOOT_DELTA_X_LIMIT;
            }
            if (delta_x > FOOT_DELTA_X_LIMIT) {
                delta_x = FOOT_DELTA_X_LIMIT;
            }
            if (delta_y < -FOOT_DELTA_Y_LIMIT) {
                delta_y = -FOOT_DELTA_Y_LIMIT;
            }
            if (delta_y > FOOT_DELTA_Y_LIMIT) {
                delta_y = FOOT_DELTA_Y_LIMIT;
            }

            state.ctrl.foot_pos_target_rel(0, i) += delta_x;
            state.ctrl.foot_pos_target_rel(1, i) += delta_y;

            state.ctrl.foot_pos_target_abs.block<3, 1>(0, i) = state.fbk.root_rot_mat * state.ctrl.foot_pos_target_rel.block<3, 1>(0, i);
            state.ctrl.foot_pos_target_world.block<3, 1>(0, i) = state.ctrl.foot_pos_target_abs.block<3, 1>(0, i) + state.fbk.root_pos;
        }

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
            }        
            for (int i = 0; i < NUM_LEG; i++)
            {
                state.ctrl.plan_contacts[i] = leg_FSM[i].get_contact_state();
            }
        }



        return true;
    }
}  // namespace legged