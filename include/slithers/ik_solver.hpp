#pragma once

/**
 * @file ik_solver.hpp
 * @brief Inverse-kinematics solver for the SLITHERS library.
 *
 * Declares IKParameters and the single entry-point ik_optimization().
 * The implementation lives in src/ik_solver.cpp.
 *
 * Algorithm summary
 * -----------------
 * The optimiser solves for a joint-velocity delta d_state that minimises
 *
 *   J(x) = || lambda_e .* (get_tau_joints(screws, x, dof, dt) - tau_goal) ||₂
 *         + || lambda_v .* x ||₂
 *
 * where tau_goal is the Lie-algebra vector of the relative goal pose
 * (expressed in the base frame).  Minimisation is performed via gradient
 * descent with central-difference gradients and a backtracking line search,
 * with box constraints enforced by clamping.
 *
 * Dependencies:
 *   - Eigen 3 (Eigen/Dense)
 *   - slithers/lie_utils.hpp  (Dof, get_tau_joints, manifold_to_vector)
 *   - C++17
 */

#include <Eigen/Dense>
#include "slithers/lie_utils.hpp"   // for slithers::Dof

namespace slithers {

// ---------------------------------------------------------------------------
// Parameter struct
// ---------------------------------------------------------------------------

/**
 * @brief Parameters for the SLITHERS IK optimisation.
 *
 * Populate once per robot/trial and reuse across time steps.
 */
struct IKParameters {
    Eigen::MatrixXd              screws;          ///< n×6 screw axes (body-frame)
    Eigen::Matrix4d              config_state;    ///< Zero-config SE(3) end-effector pose
    Eigen::Matrix4d              base_pose;       ///< Current mobile-base SE(3) pose
    Eigen::VectorXd              state_min;       ///< Joint lower limits (n)
    Eigen::VectorXd              state_max;       ///< Joint upper limits (n)
    Eigen::Matrix<double, 6, 1>  lambda_e;        ///< Cartesian error weights (6)
    double                       lambda_j{0.001}; ///< Jerk penalty weight (reserved for future use)
    Eigen::VectorXd              lambda_v;        ///< Joint velocity weights (n)
    double                       dt{0.1};         ///< Time step [s]
    slithers::Dof                dof;             ///< DOF breakdown (base + arm = total)
};

// ---------------------------------------------------------------------------
// IK solver entry point
// ---------------------------------------------------------------------------

/**
 * @brief Solve the SLITHERS IK for one time step.
 *
 * Finds joint velocity delta d_state that minimises:
 *
 *   J(x) = || lambda_e .* (get_tau_joints(screws, x, dof, dt) - tau_goal) ||₂
 *         + || lambda_v .* x ||₂
 *
 * where tau_goal = manifold_to_vector(base_pose⁻¹ * pose_goal * config_state⁻¹).
 *
 * Uses gradient descent with central finite differences and backtracking
 * line search.  Box constraints [state_min, state_max] are enforced by
 * clamping the candidate after each step.  The warm start is current_state.
 *
 * @param current_state  Current joint state (n), used as warm start.
 * @param current_vel    Current joint velocity (n).  Used only for
 *                       dimension validation in this implementation.
 * @param pose_goal      Desired end-effector SE(3) pose (world frame).
 * @param params         IK parameters (robot model + weights + limits).
 * @return               Optimised joint velocity delta (n).
 *
 * @throws std::invalid_argument on dimension mismatches in the inputs.
 */
Eigen::VectorXd ik_optimization(
    const Eigen::VectorXd& current_state,
    const Eigen::VectorXd& current_vel,
    const Eigen::Matrix4d& pose_goal,
    const IKParameters&    params);

}  // namespace slithers
