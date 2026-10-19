#include "slithers/path_generators.hpp"
#include <cmath>

namespace slithers {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/**
 * @brief Assemble an SE(3) homogeneous transform from a rotation matrix and
 *        a translation vector.
 *
 * @param R  3x3 rotation matrix
 * @param t  3-vector translation
 * @return   4x4 pose matrix:  [ R | t ]
 *                              [ 0 | 1 ]
 */
static Eigen::Matrix4d make_pose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = R;
    pose.block<3, 1>(0, 3) = t;
    return pose;
}

/**
 * @brief Convert a rotation vector (axis-angle) to a 3x3 rotation matrix
 *        using Rodrigues' formula.
 *
 * The rotation angle is the Euclidean norm of (rx, ry, rz).
 * The rotation axis  is (rx, ry, rz) / angle.
 * Returns the identity matrix when angle < 1e-9 (avoids division by zero).
 *
 * @param rx  x-component of the rotation vector
 * @param ry  y-component of the rotation vector
 * @param rz  z-component of the rotation vector
 * @return    3x3 rotation matrix
 */
static Eigen::Matrix3d rotvec_to_mat(double rx, double ry, double rz)
{
    const double angle = std::sqrt(rx * rx + ry * ry + rz * rz);

    // Return identity for negligibly small rotations
    if (angle < 1e-9) {
        return Eigen::Matrix3d::Identity();
    }

    // Unit axis
    const double ax = rx / angle;
    const double ay = ry / angle;
    const double az = rz / angle;

    const double c  = std::cos(angle);
    const double s  = std::sin(angle);
    const double mc = 1.0 - c;  // 1 − cos(angle)

    // Rodrigues: R = c·I + (1−c)·(k⊗k) + s·[k]×
    Eigen::Matrix3d R;
    R(0, 0) = c + ax * ax * mc;
    R(0, 1) = ax * ay * mc - az * s;
    R(0, 2) = ax * az * mc + ay * s;

    R(1, 0) = ay * ax * mc + az * s;
    R(1, 1) = c + ay * ay * mc;
    R(1, 2) = ay * az * mc - ax * s;

    R(2, 0) = az * ax * mc - ay * s;
    R(2, 1) = az * ay * mc + ax * s;
    R(2, 2) = c + az * az * mc;

    return R;
}

// ---------------------------------------------------------------------------
// Path generators
// ---------------------------------------------------------------------------

/**
 * @brief Generate a vertical helix path.
 *
 * Position:
 *   x = a * cos(t)
 *   y = a * sin(t)
 *   z = b * t + 0.5
 *
 * Orientation:
 *   R_permute = [[0,0,1],[1,0,0],[0,1,0]]   (column permutation)
 *   R = rotvec_to_mat(0, 0, t) * R_permute
 *
 * Produces floor(time/dt)+1 poses sampled at t = 0, dt, 2·dt, …, time.
 */
std::vector<Matrix4d> generate_helix(double a, double b, double dt, double time)
{
    // Column-permutation matrix: maps x→z, y→x, z→y axes
    Eigen::Matrix3d R_permute;
    R_permute << 0, 0, 1,
                 1, 0, 0,
                 0, 1, 0;

    const int n_steps = static_cast<int>(std::floor(time / dt)) + 1;
    std::vector<Matrix4d> poses;
    poses.reserve(n_steps);

    for (int i = 0; i < n_steps; ++i) {
        const double t = i * dt;

        // Translation along the helix
        const Eigen::Vector3d pos(a * std::cos(t),
                                  a * std::sin(t),
                                  b * t + 0.5);

        // Rotation: yaw by t around world-z, then apply axis permutation
        const Eigen::Matrix3d R = rotvec_to_mat(0.0, 0.0, t) * R_permute;

        poses.push_back(make_pose(R, pos));
    }

    return poses;
}

/**
 * @brief Generate a horizontal sine-wave path.
 *
 * Position:
 *   x = t + 2
 *   y = 0.4470  (constant)
 *   z = b * sin(π/2 · t) + 0.5
 *
 * Orientation:
 *   R_permute2 = [[-1,0,0],[0,0,1],[0,1,0]]
 *   R = rotvec_to_mat(0, 0, 0) * R_permute2   (zero rotation → just permute)
 *
 * Produces floor(time/dt)+1 poses.
 */
std::vector<Matrix4d> generate_sine(double a, double b, double dt, double time)
{
    // Suppress unused-parameter warning; 'a' is part of the public API but
    // not used in the sine-wave parametrisation.
    (void)a;

    // Axis-permutation + reflection matrix
    Eigen::Matrix3d R_permute2;
    R_permute2 << -1, 0, 0,
                   0, 0, 1,
                   0, 1, 0;

    // rotvec_to_mat(0,0,0) returns identity, so R = R_permute2
    const Eigen::Matrix3d R = rotvec_to_mat(0.0, 0.0, 0.0) * R_permute2;

    const int n_steps = static_cast<int>(std::floor(time / dt)) + 1;
    std::vector<Matrix4d> poses;
    poses.reserve(n_steps);

    for (int i = 0; i < n_steps; ++i) {
        const double t = i * dt;

        const Eigen::Vector3d pos(t + 2.0,
                                  0.4470,
                                  b * std::sin(M_PI / 2.0 * t) + 0.5);

        poses.push_back(make_pose(R, pos));
    }

    return poses;
}

/**
 * @brief Generate a horizontal helix (corkscrew advancing along x).
 *
 * Position:
 *   x = b * t + 1
 *   y = a * sin(t) + 0.5
 *   z = 0.5 * a * cos(t) + a + 0.25
 *
 * Orientation:
 *   R_permute (same as generate_helix)
 *   R = rotvec_to_mat(0, 0, 0) * R_permute   (zero rotation → just permute)
 *
 * Produces floor(time/dt)+1 poses.
 */
std::vector<Matrix4d> generate_horizontal_helix(double a, double b, double dt, double time)
{
    // Column-permutation matrix (identical to generate_helix)
    Eigen::Matrix3d R_permute;
    R_permute << 0, 0, 1,
                 1, 0, 0,
                 0, 1, 0;

    // rotvec_to_mat(0,0,0) returns identity, so R = R_permute
    const Eigen::Matrix3d R = rotvec_to_mat(0.0, 0.0, 0.0) * R_permute;

    const int n_steps = static_cast<int>(std::floor(time / dt)) + 1;
    std::vector<Matrix4d> poses;
    poses.reserve(n_steps);

    for (int i = 0; i < n_steps; ++i) {
        const double t = i * dt;

        const Eigen::Vector3d pos(b * t + 1.0,
                                  a * std::sin(t) + 0.5,
                                  0.5 * a * std::cos(t) + a + 0.25);

        poses.push_back(make_pose(R, pos));
    }

    return poses;
}

/**
 * @brief Generate a spiral-sine path.
 *
 * Position:
 *   x = t + sin(a * t)
 *   y = cos(a * t)
 *   z = b * sin(t) + 3 * b
 *
 * Orientation:
 *   R_permute2 (same as generate_sine)
 *   R = rotvec_to_mat(0, 0, -π/4) * R_permute2
 *
 * Produces floor(time/dt)+1 poses.
 */
std::vector<Matrix4d> generate_spiral_sine(double a, double b, double dt, double time)
{
    // Axis-permutation + reflection matrix (identical to generate_sine)
    Eigen::Matrix3d R_permute2;
    R_permute2 << -1, 0, 0,
                   0, 0, 1,
                   0, 1, 0;

    // Constant −45° yaw applied before the axis permutation
    const Eigen::Matrix3d R = rotvec_to_mat(0.0, 0.0, -M_PI / 4.0) * R_permute2;

    const int n_steps = static_cast<int>(std::floor(time / dt)) + 1;
    std::vector<Matrix4d> poses;
    poses.reserve(n_steps);

    for (int i = 0; i < n_steps; ++i) {
        const double t = i * dt;

        const Eigen::Vector3d pos(t + std::sin(a * t),
                                  std::cos(a * t),
                                  b * std::sin(t) + 3.0 * b);

        poses.push_back(make_pose(R, pos));
    }

    return poses;
}

}  // namespace slithers
