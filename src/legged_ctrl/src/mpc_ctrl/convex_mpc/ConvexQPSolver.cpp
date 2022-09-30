#include "mpc_ctrl/convex_mpc/ConvexQPSolver.h"


namespace legged
{
    ConvexQPSolver::ConvexQPSolver():
        state_dim_(MPC_STATE_DIM_SPARSE), 
        control_dim_(DIM_GRF),
        horizon_(PLAN_HORIZON), 
        decision_variable_size_((MPC_STATE_DIM_SPARSE + DIM_GRF)*PLAN_HORIZON), 
        dynamics_constraint_size_((MPC_STATE_DIM_SPARSE) * PLAN_HORIZON),
        friction_constraint_size_(PLAN_HORIZON * 4 * NUM_LEG), 
        bound_constraint_size_(PLAN_HORIZON * 4) {

    }     
    ConvexQPSolver::ConvexQPSolver(Eigen::VectorXd &q_weights, Eigen::VectorXd &r_weights) :
        state_dim_(MPC_STATE_DIM_SPARSE), 
        control_dim_(DIM_GRF),
        horizon_(PLAN_HORIZON), 
        decision_variable_size_((MPC_STATE_DIM_SPARSE + DIM_GRF)*PLAN_HORIZON), 
        dynamics_constraint_size_((MPC_STATE_DIM_SPARSE) * PLAN_HORIZON),
        friction_constraint_size_(PLAN_HORIZON * 4 * NUM_LEG), 
        bound_constraint_size_(PLAN_HORIZON * 4) {
        mu_ = 0.3;     
        mpc_dt_ = 0.005;

        // Initialize hessian (only once)
        hessian_.resize(decision_variable_size_, decision_variable_size_);
        Q_.diagonal() = q_weights;
        R_.diagonal() = r_weights; 

        size_t cur_index = 0; 
        bool alternate = false; 
        for(size_t i = 0; i < hessian_.cols(); i ++) {
            if(alternate == false) {
                hessian_.insert(i,i) = r_weights[cur_index];
            } else {
                hessian_.insert(i,i) = q_weights[cur_index]; 
            }
            cur_index++; 
            if(alternate == false && cur_index == DIM_GRF) {
                alternate = true; 
                cur_index = 0; 
            } else if (alternate && cur_index == MPC_STATE_DIM_SPARSE) {
                alternate = false; 
                cur_index = 0;
            }
        }
        hessian_.makeCompressed(); 

        // Initialze constraints matrix 
        Ac_.setZero(); Ad_.setZero(); 
        Bc_.setZero(); Bc_.setZero(); 
        cons_mat_.resize(dynamics_constraint_size_ + friction_constraint_size_ + bound_constraint_size_, decision_variable_size_); 
        gradient_.resize(decision_variable_size_); gradient_.setZero(); 
        lowerBound_.resize(dynamics_constraint_size_ + friction_constraint_size_ + bound_constraint_size_); lowerBound_.setZero(); 
        upperBound_.resize(dynamics_constraint_size_ + friction_constraint_size_ + bound_constraint_size_); upperBound_.setZero(); 

        // Initialize sparsity pattern for constraint matrices. https://stackoverflow.com/questions/29830299/sparse-matrix-from-triplet
        a_trp_.push_back(Eigen::Triplet<double>(0, 6, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(0, 7, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(0, 8, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(1, 6, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(1, 7, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(1, 8, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(2, 6, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(2, 7, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(2, 8, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(3, 9, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(4, 10, 1)); 
        a_trp_.push_back(Eigen::Triplet<double>(5, 11, 1)); 

        for(size_t i = 0; i < MPC_STATE_DIM_SPARSE; i++) {
            a_trp_.push_back(Eigen::Triplet<double>(i,i,1)); 
        }

        for(size_t i = 6; i < 9; i++) {
            for(size_t j = 0; j < 12; j++) {
                b_trp_.push_back(Eigen::Triplet<double>(i, j, 1)); 
            }
        }
        b_trp_.push_back(Eigen::Triplet<double>(9, 0, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(10, 1, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(11, 2, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(9, 3, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(10, 4, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(11, 5, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(9, 6, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(10, 7, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(11, 8, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(9, 9, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(10, 10, 1)); 
        b_trp_.push_back(Eigen::Triplet<double>(11, 11, 1)); 

        for(size_t i = 0; i < MPC_STATE_DIM_SPARSE; i++) {
            I_trp_.push_back(Eigen::Triplet<double>(i, i, 1)); 
        }

        // Initialize sparsity pattern for full matrices 
        b_sparsity_.push_back(create_shifted_triplet_vector(b_trp_, 0, 0)); 
        I_sparsity_.push_back(create_shifted_triplet_vector(I_trp_, 0, DIM_GRF)); 
        for(size_t i = 1; i < horizon_; i++) {
            a_sparsity_.push_back(create_shifted_triplet_vector(a_trp_, state_dim_ * (i), state_dim_ * (i-1) + control_dim_ * i ));
            b_sparsity_.push_back(create_shifted_triplet_vector(b_trp_, state_dim_ * (i), state_dim_ * (i)   + control_dim_ * i)); 
            I_sparsity_.push_back(create_shifted_triplet_vector(I_trp_, state_dim_ * (i), state_dim_ * (i)   + control_dim_ * (i+1))); 
        }
        
        for(size_t i = 0; i < b_sparsity_.size(); i++) {
            for(size_t j = 0; j < b_sparsity_[i].size(); j++) {
                cons_mat_.insert(b_sparsity_[i][j].row(), b_sparsity_[i][j].col());
                sparse_map_[pairInt(b_sparsity_[i][j].row(), b_sparsity_[i][j].col())] = 1;
            }
        }

        for(size_t i = 0; i < I_sparsity_.size(); i++) {
            for(size_t j = 0; j < I_sparsity_[i].size(); j++) {
                cons_mat_.insert(I_sparsity_[i][j].row(), I_sparsity_[i][j].col()); 
                sparse_map_[pairInt(I_sparsity_[i][j].row(), I_sparsity_[i][j].col())] = -1;
            }
        }

        for(size_t i = 0; i < a_sparsity_.size(); i++) {
            for(size_t j = 0; j < a_sparsity_[i].size(); j++) {
                cons_mat_.insert(a_sparsity_[i][j].row(), a_sparsity_[i][j].col()); 
                sparse_map_[pairInt(a_sparsity_[i][j].row(), a_sparsity_[i][j].col())] = 1;
            }
        }

        // initialize friction cone constraint (only once)
        for(size_t i = 0; i < horizon_; i++) {
            for(size_t j = 0; j < NUM_LEG; j++) {
            cons_mat_.insert(dynamics_constraint_size_ + 0 + 16*i + j*NUM_LEG, 0 + state_dim_ * i + control_dim_ * i + j*3) = 1;                 
            cons_mat_.insert(dynamics_constraint_size_ + 1 + 16*i + j*NUM_LEG, 0 + state_dim_ * i + control_dim_ * i + j*3) = 1; 
            cons_mat_.insert(dynamics_constraint_size_ + 2 + 16*i + j*NUM_LEG, 1 + state_dim_ * i + control_dim_ * i + j*3) = 1; 
            cons_mat_.insert(dynamics_constraint_size_ + 3 + 16*i + j*NUM_LEG, 1 + state_dim_ * i + control_dim_ * i + j*3) = 1; 

            cons_mat_.insert(dynamics_constraint_size_ + 0 + 16*i + j*NUM_LEG, 2 + state_dim_ * i + control_dim_ * i + j*3) =  mu_;                 
            cons_mat_.insert(dynamics_constraint_size_ + 1 + 16*i + j*NUM_LEG, 2 + state_dim_ * i + control_dim_ * i + j*3) = -mu_ ; 
            cons_mat_.insert(dynamics_constraint_size_ + 2 + 16*i + j*NUM_LEG, 2 + state_dim_ * i + control_dim_ * i + j*3) = mu_ ; 
            cons_mat_.insert(dynamics_constraint_size_ + 3 + 16*i + j*NUM_LEG, 2 + state_dim_ * i + control_dim_ * i + j*3) = -mu_ ; 
            }
        }

        // friction constraint vectors 
        for(size_t i = 0; i < horizon_ ; i++) {
            for(size_t j = 0; j < NUM_LEG; j++) {
                lowerBound_(dynamics_constraint_size_ + 0 + 16*i + j*NUM_LEG) =  0; 
                lowerBound_(dynamics_constraint_size_ + 1 + 16*i + j*NUM_LEG) = -OsqpEigen::INFTY; 
                lowerBound_(dynamics_constraint_size_ + 2 + 16*i + j*NUM_LEG) = 0;//0; 
                lowerBound_(dynamics_constraint_size_ + 3 + 16*i + j*NUM_LEG) = -OsqpEigen::INFTY; 

                upperBound_(dynamics_constraint_size_ + 0 + 16*i + j*NUM_LEG) =  OsqpEigen::INFTY; 
                upperBound_(dynamics_constraint_size_ + 1 + 16*i + j*NUM_LEG) =  0; 
                upperBound_(dynamics_constraint_size_ + 2 + 16*i + j*NUM_LEG) =  OsqpEigen::INFTY; 
                upperBound_(dynamics_constraint_size_ + 3 + 16*i + j*NUM_LEG) =  0;
            }
        }   

        // initialize bound constraint (only once)
        for(size_t i = 0; i < horizon_ ; i++) {
            for(size_t j = 0; j <NUM_LEG; j++) {
                cons_mat_.insert(dynamics_constraint_size_ + friction_constraint_size_  + i*4 + j, 2 + state_dim_ * i + control_dim_ * i + j*3) = 1; 
            }
        }

        // bound constraints 
        for(size_t i = 0; i < horizon_ ; i++) {
            for(size_t j = 0; j <NUM_LEG; j++) {
                lowerBound_(dynamics_constraint_size_ + friction_constraint_size_ + i*4 + j) = 0; 
                upperBound_(dynamics_constraint_size_ + friction_constraint_size_ + i*4 + j) = 180;
            }

            // include the gravity term
            lowerBound_(i*MPC_STATE_DIM_SPARSE + 11) = 9.8 * mpc_dt_; 
            upperBound_(i*MPC_STATE_DIM_SPARSE + 11) = lowerBound_(i*MPC_STATE_DIM_SPARSE + 11); 
        }
        cons_mat_.makeCompressed(); 
        update_cons_matrix();     

        // Initialize solver 
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);
        solver.settings()->setAbsoluteTolerance(1e-3);
        solver.settings()->setRelativeTolerance(1e-4);
        solver.settings()->setWarmStart(true);

        solver.data()->setNumberOfVariables(decision_variable_size_);
        solver.data()->setNumberOfConstraints(dynamics_constraint_size_ + friction_constraint_size_ + bound_constraint_size_);
        solver.data()->setLinearConstraintsMatrix(cons_mat_);
        solver.data()->setHessianMatrix(hessian_);
        solver.data()->setGradient(gradient_);
        solver.data()->setLowerBound(lowerBound_);
        solver.data()->setUpperBound(upperBound_);
        solver.initSolver();

    }

void ConvexQPSolver::update_B_matrix(double robot_mass, const Eigen::Matrix3d &a1_trunk_inertia, Eigen::Matrix3d root_rot_mat,
                                 Eigen::Matrix<double, 3, NUM_LEG> foot_pos) {
    // Calculate the continuous B matrix 
    Eigen::Matrix3d a1_trunk_inertia_world;
    a1_trunk_inertia_world = root_rot_mat * a1_trunk_inertia * root_rot_mat.transpose();
    for (int i = 0; i < NUM_LEG; ++i) {
        Bc_.block<3, 3>(6, 3 * i) =
                a1_trunk_inertia_world.inverse() * Utils::skew(foot_pos.block<3, 1>(0, i));
        Bc_.block<3, 3>(9, 3 * i) =
                (1 / robot_mass) * Eigen::Matrix3d::Identity();
    }

    // Discretize
    Bd_ = Bc_*mpc_dt_;
} 

void ConvexQPSolver::update_A_matrix(Eigen::Vector3d root_euler) {
    // Calculate the continuous A matrix 
    double cos_yaw = cos(root_euler[2]);
    double sin_yaw = sin(root_euler[2]);
    Eigen::Matrix3d ang_vel_to_rpy_rate;

    ang_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
                          -sin_yaw, cos_yaw, 0,
                           0, 0, 1;
    Ac_.block<3, 3>(0, 6) = ang_vel_to_rpy_rate;
    Ac_.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();

    // Discretize
    Ad_ = Eigen::Matrix<double, MPC_STATE_DIM_SPARSE, MPC_STATE_DIM_SPARSE>::Identity() + Ac_*mpc_dt_;
} 

void ConvexQPSolver::update_cons_matrix() {
    for (int k=0; k<cons_mat_.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(cons_mat_,k); it; ++it) {
            if(it.row() < dynamics_constraint_size_) { 
                it.valueRef() = sparse_map_[pairInt(it.row(), it.col())];
            }
        }
    }
    // std::cout << cons_mat_ << std::endl; 
}


std::vector<Eigen::Triplet<double>> ConvexQPSolver::create_shifted_triplet_vector(std::vector<Eigen::Triplet<double>>& triplet_vec,  int row_offset, int col_offset) {

    std::vector<Eigen::Triplet<double>> shifted_vector; 

    for(size_t i = 0; i < triplet_vec.size(); ++i) {
        shifted_vector.push_back(Eigen::Triplet<double>(triplet_vec[i].row() + row_offset, 
                                                        triplet_vec[i].col() + col_offset, 1));
    }
    return shifted_vector; 
}


void ConvexQPSolver::calc_mpc_reference(LeggedState &state) { 
    // Initial state - update the gradient and first row of the constraint matrix    
    mpc_states_ << state.fbk.root_euler[0], state.fbk.root_euler[1], state.fbk.root_euler[2],
                        state.fbk.root_pos[0], state.fbk.root_pos[1], state.fbk.root_pos[2],
                        state.fbk.root_ang_vel[0], state.fbk.root_ang_vel[1], state.fbk.root_ang_vel[2],
                        state.fbk.root_lin_vel[0], state.fbk.root_lin_vel[1], state.fbk.root_lin_vel[2];
    state.ctrl.root_lin_vel_d_world = state.fbk.root_rot_mat * state.ctrl.root_lin_vel_d_rel;
    // Future reference state 
    for(size_t i = 0; i < horizon_; ++i) {
        // Calculate predicted state 
        mpc_states_d <<
                state.ctrl.root_euler_d[0],
                state.ctrl.root_euler_d[1],
                state.fbk.root_euler[2] + state.ctrl.root_ang_vel_d_rel[2] * mpc_dt_ * (i),
                state.fbk.root_pos[0] + state.ctrl.root_lin_vel_d_world[0] * mpc_dt_ * (i),
                state.fbk.root_pos[1] + state.ctrl.root_lin_vel_d_world[1] * mpc_dt_ * (i),
                state.ctrl.root_pos_d[2],
                state.ctrl.root_ang_vel_d_rel[0],
                state.ctrl.root_ang_vel_d_rel[1],
                state.ctrl.root_ang_vel_d_rel[2],
                state.ctrl.root_lin_vel_d_world[0],
                state.ctrl.root_lin_vel_d_world[1],
                0;
        
        // Linearize about reference (calcualte A and B matrices)
        update_A_matrix(mpc_states_d.segment(0,3)); 
        update_B_matrix(state.param.robot_mass,
                        state.param.a1_trunk_inertia,
                        state.fbk.root_rot_mat,
                        state.fbk.foot_pos_world); 

        // Update the constraint matrix 
        for(size_t j = 0; j < b_sparsity_[i].size(); j++) {
            size_t r_offset = b_sparsity_[i][j].row();  
            size_t c_offset = b_sparsity_[i][j].col();
            size_t r = b_trp_[j].row(); 
            size_t c = b_trp_[j].col();
            sparse_map_[pairInt(r_offset, c_offset)] = Bd_(r, c); 
        }
        
        if( i == 0) {
            upperBound_.segment(0, 12) = -1 * Ad_ * mpc_states_; 
            upperBound_(11) = upperBound_(11) + 9.8 * mpc_dt_; 
            lowerBound_.segment(0, 12) = upperBound_.segment(0, 12); 
        } else {
            for(size_t j = 0; j < a_sparsity_[i-1].size(); j++) {
                size_t r_offset = a_sparsity_[i-1][j].row();  
                size_t c_offset = a_sparsity_[i-1][j].col();
                size_t r = a_trp_[j].row(); 
                size_t c = a_trp_[j].col();
                sparse_map_[pairInt(r_offset, c_offset)] = Ad_(r, c); 
            }
        }
        // Update the gradient
        gradient_.segment((i+1)*DIM_GRF + (i)*MPC_STATE_DIM_SPARSE, 12) = -1 * Q_ * mpc_states_d; 
    }

    update_bound_constraints(state.ctrl.plan_contacts); 
}
Eigen::Matrix<double, DIM_GRF, 1> ConvexQPSolver::compute_grfs(LeggedState& state) {
    solver.updateLinearConstraintsMatrix(cons_mat_); 
    solver.updateBounds(lowerBound_, upperBound_); 
    solver.updateGradient(gradient_); 
    solver.solve();

    Eigen::VectorXd solution = solver.getSolution(); //12x1
    if (!isnan(solution.segment<12>(0).norm())) {
        return solution.segment<12>(0); 
    } else {
        solution.setZero(); 
        return solution.segment<12>(0);
    }
}

void ConvexQPSolver::update_bound_constraints(std::array<bool, NUM_LEG>& contacts) {
    // bound constraints 
    for(size_t i = 0; i < horizon_ ; i++) {
        for(size_t j = 0; j <NUM_LEG; j++) {
            lowerBound_(dynamics_constraint_size_ + friction_constraint_size_ + i*4 + j) = 0; 
            upperBound_(dynamics_constraint_size_ + friction_constraint_size_ + i*4 + j) = contacts[j] * 180;
        }
    }
}


}  // namespace legged