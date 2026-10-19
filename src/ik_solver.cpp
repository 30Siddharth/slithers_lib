/**
 * @file ik_solver.cpp
 * @brief Full implementation of the SLITHERS inverse-kinematics solver.
 *
 * See ik_solver.hpp for the public interface and algorithmic overview.
 */

#include "slithers/ik_solver.hpp"

#include <algorithm>    // std::clamp
#include <array>        // std::array
#include <cmath>        // std::sqrt, std::isfinite
#include <stdexcept>    // std::invalid_argument

namespace slithers {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace {

/**
 * @brief Evaluate the scalar cost at a given candidate velocity vector.
 *
 * J(x) = || lambda_e .* (tau_joints(x) - tau_goal) ||₂
 *       + || lambda_v .* x ||₂
 *
 * Both terms are plain (non-squared) norms, matching the MATLAB reference.
 * Squaring them inflates the gradient enough that the fixed line-search
 * steps overshoot straight to the joint limits.
 *
 * @param x          Candidate joint velocity vector (n).
 * @param tau_goal   Target Lie-algebra pose vector (6).
 * @param params     IK parameters (screws, dof, dt, weights).
 * @return           Scalar cost value.
 */
double compute_cost(const Eigen::VectorXd&              x,
                    const Eigen::Matrix<double, 6, 1>&  tau_goal,
                    const IKParameters&                 params)
{
    // Forward kinematics residual in the Lie algebra
    const Eigen::Matrix<double, 6, 1> tau_fwd =
        get_tau_joints(params.screws, x, params.dof, params.dt);

    const Eigen::Matrix<double, 6, 1> error =
        params.lambda_e.cwiseProduct(tau_fwd - tau_goal);

    // Weighted joint velocity regularisation
    const Eigen::VectorXd reg = params.lambda_v.cwiseProduct(x);

    return error.norm() + reg.norm();
}

/**
 * @brief Clamp every element of @p x into [lo, hi] element-wise.
 *
 * @param x   Input vector (n).
 * @param lo  Lower bound vector (n).
 * @param hi  Upper bound vector (n).
 * @return    Clamped vector (n).
 */
Eigen::VectorXd clamp_vec(const Eigen::VectorXd& x,
                           const Eigen::VectorXd& lo,
                           const Eigen::VectorXd& hi)
{
    return x.cwiseMax(lo).cwiseMin(hi);
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public implementation
// ---------------------------------------------------------------------------

/**
 * @brief Solve the SLITHERS IK for one time step.
 *
 * -----------------------------------------------------------------------
 * ALGORITHM — gradient descent with backtracking line search
 * -----------------------------------------------------------------------
 *
 * 1.  Compute the goal pose in the base frame and map it to a Lie-algebra
 *     vector tau_goal:
 *
 *       tau_goal = manifold_to_vector(base_pose⁻¹ * pose_goal * config_state⁻¹)
 *
 * 2.  Define the cost function (see compute_cost() above).
 *
 * 3.  Warm-start the iterate from current_state.
 *
 * 4.  Repeat for up to max_iter = 200 iterations:
 *
 *     a) Compute the gradient via central finite differences:
 *
 *          grad(i) = [J(x + ε·eᵢ) − J(x − ε·eᵢ)] / (2ε),  ε = 1e-6
 *
 *        where eᵢ is the i-th standard basis vector.
 *
 *     b) Backtracking line search over α ∈ {1.0, 0.5, 0.1, 0.01, 0.001}:
 *
 *          x_candidate = clamp(x − α · grad, state_min, state_max)
 *          if J(x_candidate) < J(x): accept x_candidate, break
 *
 *     c) If no step size produces improvement, break early (converged or
 *        stuck at a local flat region / constraint boundary).
 *
 *     d) If ||grad||₂ < tol (= 1e-6) before the line search, break early
 *        (gradient effectively zero — first-order optimality).
 *
 * 5.  Return the final iterate x.
 *
 * Box constraints [state_min, state_max] are always satisfied by the
 * clamping in step 4b; the returned vector is guaranteed to lie within
 * the prescribed limits.
 * -----------------------------------------------------------------------
 */
Eigen::VectorXd ik_optimization(
    const Eigen::VectorXd& current_state,
    const Eigen::VectorXd& current_vel,
    const Eigen::Matrix4d& pose_goal,
    const IKParameters&    params)
{
    // ------------------------------------------------------------------
    // 0.  Input validation
    // ------------------------------------------------------------------
    const int n = params.dof.total;

    if (params.dof.total != params.dof.base + params.dof.arm) {
        throw std::invalid_argument(
            "slithers::ik_optimization: "
            "params.dof.total must equal params.dof.base + params.dof.arm.");
    }
    if (params.screws.rows() != n || params.screws.cols() != 6) {
        throw std::invalid_argument(
            "slithers::ik_optimization: "
            "params.screws must be (dof.total × 6).");
    }
    if (current_state.size() != n) {
        throw std::invalid_argument(
            "slithers::ik_optimization: "
            "current_state length must equal dof.total.");
    }
    if (current_vel.size() != n) {
        throw std::invalid_argument(
            "slithers::ik_optimization: "
            "current_vel length must equal dof.total.");
    }
    if (params.state_min.size() != n) {
        throw std::invalid_argument(
            "slithers::ik_optimization: "
            "params.state_min length must equal dof.total.");
    }
    if (params.state_max.size() != n) {
        throw std::invalid_argument(
            "slithers::ik_optimization: "
            "params.state_max length must equal dof.total.");
    }
    if (params.lambda_v.size() != n) {
        throw std::invalid_argument(
            "slithers::ik_optimization: "
            "params.lambda_v length must equal dof.total.");
    }

    // ------------------------------------------------------------------
    // 1.  Compute the target Lie-algebra pose vector
    //
    //     Express the goal relative to the mobile base and strip the
    //     zero-config offset so that the optimiser targets a delta in
    //     the robot's local joint space.
    //
    //     tau_goal = log(base_pose⁻¹ * pose_goal * config_state⁻¹)
    // ------------------------------------------------------------------
    const Eigen::Matrix4d relative_pose =
        params.base_pose.inverse() * pose_goal * params.config_state.inverse();

    const Eigen::Matrix<double, 6, 1> tau_goal = manifold_to_vector(relative_pose);

    // ------------------------------------------------------------------
    // 2.  Warm start — begin from the current joint velocity
    // ------------------------------------------------------------------
    Eigen::VectorXd x = current_state;

    // Clamp warm start to the feasible box immediately
    x = clamp_vec(x, params.state_min, params.state_max);

    // ------------------------------------------------------------------
    // 3.  Gradient descent with central finite differences and
    //     backtracking line search
    // ------------------------------------------------------------------
    constexpr int    max_iter = 200;
    constexpr double tol      = 1e-6;  // gradient 2-norm threshold
    constexpr double eps      = 1e-6;  // finite-difference step size

    // Candidate step sizes for the backtracking line search (tried in order)
    constexpr std::array<double, 5> alphas = {1.0, 0.5, 0.1, 0.01, 0.001};

    double cost_x = compute_cost(x, tau_goal, params);

    for (int iter = 0; iter < max_iter; ++iter) {
        // ----------------------------------------------------------------
        // 3a.  Central-difference gradient
        //
        //   grad(i) = [J(x + ε·eᵢ) − J(x − ε·eᵢ)] / (2ε)
        // ----------------------------------------------------------------
        Eigen::VectorXd grad(n);
        for (int i = 0; i < n; ++i) {
            Eigen::VectorXd x_fwd = x;
            Eigen::VectorXd x_bwd = x;
            x_fwd(i) += eps;
            x_bwd(i) -= eps;
            grad(i) = (compute_cost(x_fwd, tau_goal, params)
                     - compute_cost(x_bwd, tau_goal, params))
                     / (2.0 * eps);
        }

        // Early termination when the gradient is essentially zero
        if (grad.norm() < tol) {
            break;
        }

        // ----------------------------------------------------------------
        // 3b.  Backtracking line search
        //
        //   For α in {1.0, 0.5, 0.1, 0.01, 0.001}:
        //     x_candidate = clamp(x − α·grad, state_min, state_max)
        //     if J(x_candidate) < J(x): accept, break
        // ----------------------------------------------------------------
        bool accepted = false;
        for (const double alpha : alphas) {
            const Eigen::VectorXd x_candidate =
                clamp_vec(x - alpha * grad, params.state_min, params.state_max);

            const double cost_candidate = compute_cost(x_candidate, tau_goal, params);
            if (cost_candidate < cost_x) {
                x       = x_candidate;
                cost_x  = cost_candidate;
                accepted = true;
                break;
            }
        }

        // ----------------------------------------------------------------
        // 3c.  If no step produced improvement, break early
        //
        //   This can happen when the iterates are at a local flat region,
        //   at the boundary of the box constraints, or the cost landscape
        //   around the current point does not yield descent for any of the
        //   tried step sizes.
        // ----------------------------------------------------------------
        if (!accepted) {
            break;
        }
    }

    // ------------------------------------------------------------------
    // 4.  Return the optimised joint velocity delta
    // ------------------------------------------------------------------
    return x;
}

}  // namespace slithers
