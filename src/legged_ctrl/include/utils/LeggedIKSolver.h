// This class is for calculate inverse kinematics
// we init it once to save a joint angle value to warm start the next calculation
#pragma once
#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

class LeggedIKSolver {
    public:
        static std::unique_ptr<LeggedIKSolver> createLeggedIKSolver(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile);
        LeggedIKSolver(PinocchioInterface& pino_interface, CentroidalModelInfo& info,
            const PinocchioEndEffectorKinematics& ee_kinematics);

        // leg ID, follows ETH convension 0-LF 1-LH 2-RF 3-RH, notice this is different from unitree conversion
        void setWarmStartPos(vector_t prev_joint_angs, int leg_id);
        void setWarmStartPos(vector_t prev_joint_angs4);
        void setBasePos(vector_t orientation_pos);
        // solve ik for individual leg
        vector_t solveIK(vector3_t foot_pos, int leg_id);
        
    private:
        PinocchioInterface& pino_interface_;
        const CentroidalModelInfo& info_;
        PinocchioCentroidalDynamics centroidal_dynamics_;
        CentroidalModelPinocchioMapping mapping_;
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;

        // set pinocchio model
        vector_t state_q, state_v; 
        // save joint angles we want to solve 
        vector_t joint_angs4_;
        // save previous joint angles for warm starting the solve
        vector_t prev_joint_angs4_;
        // save target foot positions we want to reach
        vector_t foot_pos4_;

        // IK parameters 
        const double EPS  = 1e-4;
        const int IT_MAX  = 50;
        const double DT   = 1e-3;
        const double DAMP = 1e-9;
};

}// namespace legged