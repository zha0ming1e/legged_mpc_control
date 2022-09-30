//
// Created by shuoy on 10/19/21.
//

#include "utils/Utils.h"

Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y*y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}

Eigen::Vector3d Utils::quatToZyx(const Eigen::Quaterniond q)
{
  Eigen::Matrix<double, 3, 1> zyx;

  double as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) =
      std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) =
      std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

Eigen::Vector3d Utils::quat_to_rotVec(Eigen::Quaterniond quat) {
    Eigen::Vector3d axis; axis.setZero(); 
    double angle = 0.0; 

    Eigen::Vector3d vec{quat.x(), quat.y(), quat.z()};  
    if(vec.norm() < 1e-12) {
        axis.setZero(); 
    } else {
        axis = vec / vec.norm(); 
    }
    angle = 2 * atan2(vec.norm(), quat.w()); 

    return axis * angle; 

}

Eigen::Matrix3d Utils::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;
}

// https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
Eigen::Matrix3d Utils::pseudo_inverse(const Eigen::Matrix3d &mat) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double epsilon = std::numeric_limits<double>::epsilon();
    // For a non-square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
           svd.matrixU().adjoint();
}

double Utils::cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2) {
    // surface 1: a1 * x + b1 * y + c1 * z + d1 = 0, coef: [a1, b1, c1]
    // surface 2: a2 * x + b2 * y + c2 * z + d2 = 0, coef: [a1, b2, c2]
    double angle_cos =
            abs(surf_coef_1[0] * surf_coef_2[0] + surf_coef_1[1] * surf_coef_2[1] + surf_coef_1[2] * surf_coef_2[2])
            / (sqrt(surf_coef_1[0] * surf_coef_1[0] + surf_coef_1[1] * surf_coef_1[1] + surf_coef_1[2] * surf_coef_1[2]) *
               sqrt(surf_coef_2[0] * surf_coef_2[0] + surf_coef_2[1] * surf_coef_2[1] + surf_coef_2[2] * surf_coef_2[2]));
    return acos(angle_cos);
}

Eigen::Matrix<double, NUM_DOF,1> Utils::joint_vec_unitree_to_pinnochio(Eigen::Matrix<double, NUM_DOF,1> vec) {
    Eigen::Matrix<double, NUM_DOF,1> return_vec;
    return_vec.segment<3>(0) = vec.segment<3>(0);
    return_vec.segment<3>(3) = vec.segment<3>(6);
    return_vec.segment<3>(6) = vec.segment<3>(3);
    return_vec.segment<3>(9) = vec.segment<3>(9);
    return return_vec;
}

Eigen::Matrix<double, NUM_DOF,1> Utils::joint_vec_pinnochio_to_unitree(Eigen::Matrix<double, NUM_DOF,1> vec) {
    Eigen::Matrix<double, NUM_DOF,1> return_vec;
    return_vec.segment<3>(0) = vec.segment<3>(0);
    return_vec.segment<3>(3) = vec.segment<3>(6);
    return_vec.segment<3>(6) = vec.segment<3>(3);
    return_vec.segment<3>(9) = vec.segment<3>(9);
    return return_vec;
}

Eigen::Matrix<double, 6,1> BezierUtils::get_foot_pos_curve(float t,
                                   Eigen::Vector3d foot_pos_start,
                                   Eigen::Vector3d foot_pos_final, 
                                   double terrain_pitch_angle = 0)
{
    Eigen::Matrix<double, 6,1> foot_pos_vel_target;
    Eigen::Vector2d rst;
    // X-axis
    std::vector<double> bezierX{foot_pos_start(0),
                               foot_pos_start(0),
                                foot_pos_final(0),
                                foot_pos_final(0),
                                foot_pos_final(0)};
    rst = bezier_curve(t, bezierX);
    foot_pos_vel_target[0]  = rst[0];
    foot_pos_vel_target[3]  = rst[1];

    // Y-axis
    std::vector<double> bezierY{foot_pos_start(1),
                                foot_pos_start(1),
                                foot_pos_final(1),
                                foot_pos_final(1),
                                foot_pos_final(1)};
    rst = bezier_curve(t, bezierY);
    foot_pos_vel_target[1]  = rst[0];
    foot_pos_vel_target[4]  = rst[1];

    // Z-axis
    std::vector<double> bezierZ{foot_pos_start(2),
                                foot_pos_start(2),
                                foot_pos_final(2),
                                foot_pos_final(2),
                                foot_pos_final(2)};
    bezierZ[1] += FOOT_SWING_CLEARANCE1;
    bezierZ[2] += FOOT_SWING_CLEARANCE2 + 0.5*sin(terrain_pitch_angle);
    rst = bezier_curve(t, bezierZ);
    foot_pos_vel_target[2]  = rst[0];
    foot_pos_vel_target[5]  = rst[1];

    return foot_pos_vel_target;
}


Eigen::Vector2d BezierUtils::bezier_curve(double t, const std::vector<double> &P) {
    double y = 0, dy = 0;
    // for (int i = 0; i <= bezier_degree; i++) {
    //     y += bezier_coefficient(t, bezier_degree, i) * P[i];
    //     if (i < bezier_degree) {
    //         dy += bezier_coefficient(t, bezier_degree - 1, i) * (P[i + 1] - P[i]) * bezier_degree;
    //     }
    // }
    std::vector<double> coefficients{1, 4, 6, 4, 1};
    for (int i = 0; i <= bezier_degree; i++) {
        y += coefficients[i] * std::pow(t, i) * std::pow(1 - t, bezier_degree - i) * P[i];
    }
    return Eigen::Vector2d(y, dy);
}

double BezierUtils::binomial(int n, int k) {
    if (k > n) {
        return 0;
    }
    if (k == 0 || k == n) {
        return 1;
    }
    return binomial(n - 1, k - 1) + binomial(n - 1, k);
}

double BezierUtils::bezier_coefficient(double t, int n, int k) {
    return binomial(n, k) * std::pow(t, k) * std::pow(1 - t, n - k);
}
