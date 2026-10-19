#pragma once
#include <Eigen/Dense>

namespace slithers {

using Matrix4d = Eigen::Matrix4d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

/// Holds all kinematic data for a robot model.
struct RobotModel {
    MatrixXd screws;        ///< n×6 screw axes, each row = [omega(3); v(3)]
    Matrix4d config_state;  ///< Zero-configuration SE(3) pose of the end-effector
    VectorXd state_min;     ///< Lower joint limits (n)
    VectorXd state_max;     ///< Upper joint limits (n)
    int base_dof = 0;
    int arm_dof  = 0;
};

/// Husky + UR5e (differential base, 8 DOF total: 2 base + 6 arm)
RobotModel husky_ur5e();

/// Husky + UR5e holonomic base (9 DOF total: 3 base + 6 arm)
RobotModel husky_ur5e_holo();

/// Reduced 4-DOF robot (1 base + 3 arm), used for testing
RobotModel reduced_robot();

}  // namespace slithers
