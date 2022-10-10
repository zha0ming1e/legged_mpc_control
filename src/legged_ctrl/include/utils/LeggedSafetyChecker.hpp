#pragma once
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "LeggedParams.h"
#include "LeggedState.h"
#include "utils/Utils.h"

namespace legged
{

    class LeggedSafetyChecker
    {
        public:
            LeggedSafetyChecker() {};

            bool is_safe(LeggedState &legged_state) {
                // safe rules
                if (legged_state.fbk.root_euler[0] > 1 || legged_state.fbk.root_euler[0] < -1) {
                    std::cout << "LeggedSafetyChecker: robot roll is too large " << std::endl;
                    return false;
                }
                if (legged_state.fbk.root_euler[1] > 1 || legged_state.fbk.root_euler[1] < -1) {
                    std::cout << "LeggedSafetyChecker: robot pitch is too large " << std::endl;
                    return false;
                }
<<<<<<< HEAD
                if (legged_state.fbk.joint_vel.maxCoeff() > 30.0) {
=======
                if (legged_state.fbk.joint_vel.maxCoeff() > 25.0) {
>>>>>>> 8067aaad9fe48aabf03b7fcd1f642776cd6710a4
                    std::cout << "LeggedSafetyChecker: joint velocity is too large " << std::endl;
                    return false;
                }
                return true;
            }

    };
}   // namespace legged