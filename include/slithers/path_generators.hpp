#pragma once
#include <Eigen/Dense>
#include <vector>

namespace slithers {

using Matrix4d = Eigen::Matrix4d;

/**
 * @brief Generate a vertical helix path.
 * @param a  Radius
 * @param b  Vertical rise per radian
 * @param dt Time step [s]
 * @param time Total duration [s]
 * @return Vector of SE(3) poses along the path
 */
std::vector<Matrix4d> generate_helix(double a, double b, double dt, double time);

/// Generate a horizontal sine-wave path.
std::vector<Matrix4d> generate_sine(double a, double b, double dt, double time);

/// Generate a horizontal helix (cork-screw advancing in x).
std::vector<Matrix4d> generate_horizontal_helix(double a, double b, double dt, double time);

/// Generate a spiral-sine path.
std::vector<Matrix4d> generate_spiral_sine(double a, double b, double dt, double time);

}  // namespace slithers
