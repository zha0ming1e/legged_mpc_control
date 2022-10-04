#include <Eigen/Dense>
#include <chrono>
#include <iostream>

#include "../../include/utils/A1Kinematics.h"

int main(int, char **) {
    legged::A1Kinematics test_kin;

    // generate some theta value
    srand((unsigned)time(NULL));
    Eigen::Vector3d q;
    // q1: [-46, 46] deg = [-0.8, 0.8] rad
    // q2: [-60, 240] deg = [-1.05, 4.19] rad
    // q3: [-154.5, -52.5] = [-2.69, -0.92] rad
    q << (double)rand() / RAND_MAX * PI / 180 * 46 * 2 - PI / 180 * 46,
         (double)rand() / RAND_MAX * PI / 180 * 300 - PI / 180 * 60,
         (double)rand() / RAND_MAX * PI / 180 * 207 - PI / 180 * 52.5;
    std::cout << "q: " << std::endl;
    std::cout << q << std::endl << std::endl;

    // define physical params for FL
    double ox = 0.1805;
    double oy = 0.047;
    double d = 0.0838;
    double lt = 0.21;
    double lc = 0.21;

    Eigen::VectorXd rho_fix(5);
    rho_fix << ox, oy, d, lt, lc;

    Eigen::VectorXd rho_opt(3);
    rho_opt << 0.0, 0.0, 0.0;

    // calculate fk
    Eigen::Vector3d foot_pos;
    foot_pos = test_kin.fk(q, rho_opt, rho_fix);

    // calculate ik
    Eigen::Vector3d inv_kin_sol;
    inv_kin_sol = test_kin.inv_kin(foot_pos, rho_opt, rho_fix);

    std::cout << std::endl << "inv_kin_sol: " << std::endl;
    std::cout << inv_kin_sol << std::endl;

    return 0;
}