#include "mpc_ctrl/ci_mpc/LciMpc.h"

namespace legged
{

LciMpc::LciMpc() {
    // Initialize Julia with sysimage
    jl_init_with_image("/usr/local/julia/bin", "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/src/mpc_ctrl/ci_mpc/lib/EmbeddedLciMpc.jl/LciMPCSysImageDocker.so");
    jl_eval_string("cd(\"/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/src/mpc_ctrl/ci_mpc/lib/EmbeddedLciMpc.jl\")");
    jl_eval_string("using Pkg");
    jl_eval_string("Pkg.activate(\".\")"); 
    jl_eval_string("using EmbeddedLciMpc");


    jl_eval_string("include(\"scripts/stand_policy.jl\")");
    jl_eval_string("include(\"scripts/trot_policy.jl\")");
    jl_eval_string("include(\"scripts/wall_stand_policy.jl\")");
    jl_eval_string("include(\"scripts/wall_walk_policy.jl\")");
    
    // Obtain mpc module from Julia 
    julia_mpc_module_ = (jl_module_t *)jl_eval_string("EmbeddedLciMpc");

    // Obtain the necessary function 
    standing_policy_ = (jl_value_t*) jl_eval_string("p_stand");
    // standing_policy_ = (jl_value_t*) jl_eval_string("tvlqr_policy");
    // walking_policy_ = (jl_value_t*) jl_eval_string("p_walk"); 
    // walking_policy_ = (jl_value_t*) jl_eval_string("p_wall");
    wall_climb_policy_ = (jl_value_t*) jl_eval_string("p_wall_stand");
    wall_walk_policy_ = (jl_value_t*) jl_eval_string("p_wall_walk");
    policy_function_ = jl_get_function(julia_mpc_module_, "exec_policy");
    // update_velocity_function_ = jl_get_function(jl_main_module, "update_velocity");

    // Populate the arguments. Modify the element during calls to update calls 
    JuliaVector1d x(Eigen::VectorXd(40)); 
    JuliaVector1d v_des(Eigen::VectorXd(6)); 
    policy_args_.push_back((jl_value_t*) standing_policy_);
    policy_args_.push_back((jl_value_t*) x.toJArray());
    policy_args_.push_back((jl_value_t*) jl_box_float64(0.0)); 
    velocity_args_.push_back((jl_value_t*) standing_policy_); 
    velocity_args_.push_back((jl_value_t*) JuliaVector1d(Eigen::VectorXd(6)).toJArray()); 

    // initialize velocity filter 
    for (int i = 0; i < 18; ++i) {
        pos_filter_bank[i] = MovingWindowFilter(5);
        vel_filter_bank[i] = MovingWindowFilter(5);
    }
}

double startTime = -1;
double curMoveMode = 0;
bool LciMpc::update(LeggedState &legged_state, double t, double dt) {
    if (legged_state.estimation_inited == false) {
        std::cout << "the estimator in the low level loop is not inited properly!" << std::endl;
        return true;
    }

    if (startTime == -1) {
        startTime = t;
        curMoveMode = legged_state.ctrl.movement_mode;
    }

    if (curMoveMode != legged_state.ctrl.movement_mode) {
        startTime = t;
        curMoveMode = legged_state.ctrl.movement_mode;
    }

    // Populate state for the robot 
    // p : [position; rotation vector; FL foot; FR foot; RL foot ; RR foot]
    // v : [velocity; angular velocity; FL foot fel; FR foot vel, RL foot vel; RR foot vel]
    Eigen::Matrix<double, 18,1> position;  position.setZero();
    Eigen::Matrix<double, 18,1> velocity;  velocity.setZero();
    Eigen::Matrix<double, 40,1> x; x.setZero();
    position.segment<3>(0) = legged_state.fbk.root_pos; 
    position.segment<3>(3) = legged_state.fbk.root_euler; //Utils::quat_to_rotVec(legged_state.root_quat); pitch roll yaw
    velocity.segment<3>(0) = legged_state.fbk.root_lin_vel; 
    velocity.segment<3>(3) = legged_state.fbk.root_ang_vel; 

    // populate foot positions 
    size_t index_offset = 6; 
    for(size_t i = 0; i < NUM_LEG; ++i) {
        position.segment<3>(3*i + index_offset) = legged_state.fbk.foot_pos_world.block<3,1>(0,i); 
        velocity.segment<3>(3*i + index_offset) = legged_state.fbk.foot_vel_world.block<3,1>(0,i); 
    }
    // // filter velocity
    Eigen::Matrix<double, 18,1> velocity_filtered; 
    velocity_filtered.segment<6>(0) = velocity.segment<6>(0);
    Eigen::Matrix<double, 18,1> position_filtered;  
    position_filtered.segment<6>(0) = position.segment<6>(0);
    for (int i = 0; i < 12; ++i) {
        velocity_filtered[index_offset+i] = 
            vel_filter_bank[index_offset+i].CalculateAverage(velocity[index_offset+i]);
        position_filtered[index_offset+i] = 
            pos_filter_bank[index_offset+i].CalculateAverage(position[index_offset+i]);
    }

    x.segment<36>(0) << position_filtered, velocity_filtered;
    // x.segment<36>(0) << position, velocity;
    x.segment<4>(36) << legged_state.fbk.foot_force_sensor; 

    // Call LciMPC policy: switch policy depending on movement mode 
    if(legged_state.ctrl.movement_mode == 0) {
        policy_args_[0] = (jl_value_t*) standing_policy_; 
        velocity_args_[0]= policy_args_[0]; 
    } else if(legged_state.ctrl.movement_mode == 1) {
        policy_args_[0] = (jl_value_t*) wall_climb_policy_; 
        // policy_args_[0] = (jl_value_t*) walking_policy_; 
        velocity_args_[0]= policy_args_[0]; 
    } else {
        policy_args_[0] = (jl_value_t*) wall_walk_policy_; 
        velocity_args_[0]= policy_args_[0]; 
    }

    // update the reference velocity
    Eigen::VectorXd v_des(6); 
    v_des << legged_state.ctrl.root_lin_vel_d_rel, legged_state.ctrl.root_ang_vel_d_rel; 
    JuliaVector1d v_des_jl(v_des); 
    // std::cout << v_des.transpose() << std::endl; 
    velocity_args_[1] = (jl_value_t*) v_des_jl.toJArray(); 
    // jl_call(update_velocity_function_, velocity_args_.data(), velocity_args_.size()); 

    // call the lci policy 
    JuliaVector1d x_jl(x); 
    policy_args_[1] = (jl_value_t*) x_jl.toJArray(); 
    policy_args_[2] = (jl_value_t*) jl_box_float64(t - startTime); 
    JuliaVector1d result((jl_array_t*) jl_call(policy_function_, policy_args_.data(), policy_args_.size()));

    // Lci Mpc wrapper output [control_input; state_next; v_next]
    Eigen::VectorXd output = result.toEigenVector(); 
    Eigen::Matrix<double, 12,1> foot_forces = output.segment<12>(0); 
    Eigen::Matrix<double, 18,1> state_des = output.segment<18>(12); 
    Eigen::Matrix<double, 18,1> vel_des = output.segment<18>(30); 
    Eigen::Matrix<double, 18,1> state_ref = output.segment<18>(48); 
    Eigen::Matrix<double, 18,1> vel_ref = output.segment<18>(56); 

    // convert state_des and vel_des to legged_state.ctrl.optimized_state and legged_state.ctrl.optimized_input
    // legged_state.ctrl.optimized_state [position, euler(ZYX), joint position]
    // legged_state.ctrl.optimized_input [foot grf; foot world velocity]
    legged_state.ctrl.optimized_state.segment<3>(0) = state_des.segment<3>(0);
    // reverse the euler angle direction 
    legged_state.ctrl.optimized_state.segment<3>(3) = Eigen::Vector3d(state_des(5), state_des(4), state_des(3));
    // legged_state.ctrl.optimized_state.segment<3>(3) = state_des.segment<3>(3);
    // legged_state.ctrl.optimized_state.segment<3>(3) = Eigen::Vector3d::Zero();
    legged_state.ctrl.optimized_state.tail(12) = state_des.tail(12);

    legged_state.ctrl.optimized_input.head(12) = foot_forces;
    legged_state.ctrl.optimized_input.tail(12) = vel_des.tail(12);

    // test 
    // legged_state.ctrl.optimized_state << 0, 0, 0.3,
    //                                      0, 0,   0,//zyx
    //                                      0.18, 0.17, 0,
    //                                      0.18, -0.17, 0, 
    //                                      -0.18, 0.17, 0, 
    //                                      -0.18, -0.17, 0;
    // legged_state.ctrl.optimized_input << 0,0, 12*9.8/4,
    //                                      0,0, 12*9.8/4,
    //                                      0,0, 12*9.8/4,
    //                                      0,0, 12*9.8/4,
    //                                      0,0,0,
    //                                      0,0,0,
    //                                      0,0,0,
    //                                      0,0,0;       

    // set contacts according to contact flag thresholding 

    for(size_t i = 0; i < NUM_LEG; ++i) {
        // this estimated_contacts is generated by the KF
        legged_state.ctrl.plan_contacts[i] = legged_state.fbk.estimated_contacts[i];
    }

    // TODO: set this flag to false if the MPC solver fails
    legged_state.mpc_solver_inited = true;
    return true; 
}

}  // namespace legged