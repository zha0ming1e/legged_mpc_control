
#pragma once
#include <vector>
#include <chrono>

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include <unordered_map> 

#include "LeggedParams.h"
#include "LeggedState.h"
#include "utils/Utils.h"

class ConvexQPSolver {
typedef std::pair<int, int> pairInt; 
public: 
    ConvexQPSolver(); 

    ConvexQPSolver(Eigen::VectorXd &q_weights_, Eigen::VectorXd &r_weights_); 

    void update_A_matrix(Eigen::Vector3d root_euler); 

    void update_B_matrix(double robot_mass, const Eigen::Matrix3d &a1_trunk_inertia, Eigen::Matrix3d root_rot_mat,
                         Eigen::Matrix<double, 3, NUM_LEG> foot_pos); 

    void update_cons_matrix(); 

    void calc_mpc_reference(A1CtrlStates& state); 

    Eigen::Matrix<double, DIM_GRF, 1> compute_grfs(A1CtrlStates& state); 

    void update_bound_constraints(bool contacts[NUM_LEG]);

    std::vector<Eigen::Triplet<double>> create_shifted_triplet_vector(std::vector<Eigen::Triplet<double>>& triplet_vec,  int row_offset, int col_offset); 

private: 
    size_t horizon_; 
    size_t state_dim_; 
    size_t control_dim_; 
    size_t decision_variable_size_; 
    size_t dynamics_constraint_size_; 
    size_t friction_constraint_size_;
    size_t bound_constraint_size_; 
    double mu_; 
    double mpc_dt_;
    Eigen::Matrix<double, MPC_STATE_DIM_SPARSE, MPC_STATE_DIM_SPARSE> Ac_;
    Eigen::Matrix<double, MPC_STATE_DIM_SPARSE, MPC_STATE_DIM_SPARSE> Ad_;
    Eigen::Matrix<double, MPC_STATE_DIM_SPARSE, DIM_GRF> Bc_;
    Eigen::Matrix<double, MPC_STATE_DIM_SPARSE, DIM_GRF> Bd_;
    Eigen::DiagonalMatrix<double, MPC_STATE_DIM_SPARSE> Q_;
    Eigen::DiagonalMatrix<double, DIM_GRF> R_;
    Eigen::Matrix<double, MPC_STATE_DIM_SPARSE, 1> mpc_states_; 
    Eigen::Matrix<double, MPC_STATE_DIM_SPARSE, 1> mpc_states_d; 


    OsqpEigen::Solver solver;
    Eigen::VectorXd lowerBound_; 
    Eigen::VectorXd upperBound_; 
    Eigen::VectorXd gradient_; 
    Eigen::SparseMatrix<double, Eigen::ColMajor> hessian_; 
    Eigen::SparseMatrix<double, Eigen::ColMajor> cons_mat_;

    // Sparsity 
    std::vector<Eigen::Triplet<double>> a_trp_; // sparsity pattern for A with indices at    0,0
    std::vector<Eigen::Triplet<double>> b_trp_; // sparsity pattern for B with its origin at 0,0
    std::vector<Eigen::Triplet<double>> I_trp_; // sparsity pattern for I with its origin at 0,0
    std::vector<std::vector<Eigen::Triplet<double>>> a_sparsity_;  // initialize sparsity pattern for Bs in the constraint matrix
    std::vector<std::vector<Eigen::Triplet<double>>> b_sparsity_;  // initialize sparsity pattern for As in the constraint matrix
    std::vector<std::vector<Eigen::Triplet<double>>> I_sparsity_;  // initialize sparsity pattern for Is in the constraint matrix

    struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2;  
    }
    };

    std::unordered_map<std::pair<int, int>, double, pair_hash> sparse_map_; 
}; 

#endif //A1_CPPFASTCONVEX_H