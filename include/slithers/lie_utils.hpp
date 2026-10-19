#pragma once

/**
 * @file lie_utils.hpp
 * @brief Core Lie theory math layer for SLITHERS.
 *
 * Provides header-only utilities for working with elements of SE(3) and its
 * Lie algebra se(3), including conversion between vector, algebra (4×4 matrix),
 * and manifold (homogeneous transform) representations, as well as forward-step
 * composition for multi-DOF screw-based kinematics.
 *
 * All functions are inline and reside in namespace slithers.
 *
 * Dependencies:
 *   - Eigen 3 (Eigen/Dense)
 *   - Eigen unsupported MatrixFunctions (for matrix exp/log)
 *   - C++17
 */

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <stdexcept>

namespace slithers {

// ---------------------------------------------------------------------------
// Fundamental types
// ---------------------------------------------------------------------------

using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

/**
 * @brief Describes the degree-of-freedom split between a floating base and an
 *        articulated arm.
 *
 * The invariant `total == base + arm` must hold; it is checked at runtime by
 * functions that use this struct.
 */
struct Dof {
    int base  = 0; ///< Number of base (floating-body) DOFs.
    int arm   = 0; ///< Number of arm (joint) DOFs.
    int total = 0; ///< Must equal base + arm.
};

// ---------------------------------------------------------------------------
// se(3) ↔ ℝ⁶  conversions
// ---------------------------------------------------------------------------

/**
 * @brief Extract the ℝ⁶ twist coordinates from a 4×4 se(3) algebra element.
 *
 * The algebra element has the structure:
 * @verbatim
 *   [  0  -ωz   ωy  | ρx ]
 *   [  ωz   0  -ωx  | ρy ]
 *   [ -ωy  ωx   0   | ρz ]
 *   [  0    0    0   |  0 ]
 * @endverbatim
 *
 * Extraction convention:
 *   - v(0) = algebra(2,1)  →  ωx
 *   - v(1) = algebra(0,2)  →  ωy
 *   - v(2) = algebra(1,0)  →  ωz
 *   - v(3) = algebra(0,3)  →  ρx
 *   - v(4) = algebra(1,3)  →  ρy
 *   - v(5) = algebra(2,3)  →  ρz
 *
 * @param algebra A 4×4 se(3) matrix (skew-symmetric upper-left 3×3 block,
 *                translation in the last column, zero bottom row).
 * @return Six-dimensional twist vector [ωx, ωy, ωz, ρx, ρy, ρz]ᵀ.
 */
inline Vector6d algebra_to_vector(const Matrix4d& algebra)
{
    Vector6d v;
    // Angular part — extracted from the skew-symmetric block
    v(0) = algebra(2, 1); // ωx
    v(1) = algebra(0, 2); // ωy
    v(2) = algebra(1, 0); // ωz
    // Linear part — last column, first three rows
    v(3) = algebra(0, 3); // ρx
    v(4) = algebra(1, 3); // ρy
    v(5) = algebra(2, 3); // ρz
    return v;
}

/**
 * @brief Build a 4×4 se(3) algebra element from a six-dimensional twist vector.
 *
 * The inverse of algebra_to_vector().  Given v = [ωx, ωy, ωz, ρx, ρy, ρz]ᵀ
 * the resulting matrix is:
 * @verbatim
 *   [  0  -ωz   ωy  | ρx ]
 *   [  ωz   0  -ωx  | ρy ]
 *   [ -ωy  ωx   0   | ρz ]
 *   [  0    0    0   |  0 ]
 * @endverbatim
 *
 * @param vec Six-dimensional twist vector [ωx, ωy, ωz, ρx, ρy, ρz]ᵀ.
 * @return 4×4 se(3) matrix.
 */
inline Matrix4d vector_to_algebra(const Vector6d& vec)
{
    Matrix4d A = Matrix4d::Zero();
    // Skew-symmetric block for angular velocity
    A(0, 1) = -vec(2); // -ωz
    A(0, 2) =  vec(1); //  ωy
    A(1, 0) =  vec(2); //  ωz
    A(1, 2) = -vec(0); // -ωx
    A(2, 0) = -vec(1); // -ωy
    A(2, 1) =  vec(0); //  ωx
    // Translation column
    A(0, 3) = vec(3); // ρx
    A(1, 3) = vec(4); // ρy
    A(2, 3) = vec(5); // ρz
    // Bottom row remains zero (already zero-initialised)
    return A;
}

// ---------------------------------------------------------------------------
// Algebra ↔ Manifold (SE(3)) conversions
// ---------------------------------------------------------------------------

/**
 * @brief Map a 4×4 se(3) algebra element to its corresponding SE(3) element
 *        via the matrix exponential.
 *
 * Uses Eigen's unsupported MatrixFunctions module for the computation, which
 * implements a Padé-approximant-based algorithm suitable for general matrices.
 *
 * @param algebra A 4×4 se(3) matrix.
 * @return 4×4 SE(3) homogeneous transformation matrix.
 */
inline Matrix4d algebra_to_manifold(const Matrix4d& algebra)
{
    return algebra.exp();
}

/**
 * @brief Map a 4×4 SE(3) element back to its se(3) algebra element via the
 *        matrix logarithm.
 *
 * Uses Eigen's unsupported MatrixFunctions module.  The result is only
 * well-defined when @p manifold is in the principal branch of the logarithm
 * (rotation angle in (-π, π)).
 *
 * @param manifold A 4×4 SE(3) homogeneous transformation matrix.
 * @return 4×4 se(3) matrix (the principal logarithm).
 */
inline Matrix4d manifold_to_algebra(const Matrix4d& manifold)
{
    return manifold.log();
}

// ---------------------------------------------------------------------------
// Vector ↔ Manifold convenience wrappers
// ---------------------------------------------------------------------------

/**
 * @brief Convert a twist vector scaled by a state increment to an SE(3)
 *        element.
 *
 * Equivalent to:
 * @code
 *   algebra_to_manifold(vector_to_algebra(vec) * d_state)
 * @endcode
 *
 * @param vec     Six-dimensional twist vector [ωx, ωy, ωz, ρx, ρy, ρz]ᵀ.
 * @param d_state Scalar multiplier applied to the algebra element before
 *                exponentiation (e.g. a time step or joint displacement).
 * @return 4×4 SE(3) homogeneous transformation matrix.
 */
inline Matrix4d vector_to_manifold(const Vector6d& vec, double d_state)
{
    return algebra_to_manifold(vector_to_algebra(vec) * d_state);
}

/**
 * @brief Convert an SE(3) element to a six-dimensional twist vector.
 *
 * Equivalent to:
 * @code
 *   algebra_to_vector(manifold_to_algebra(manifold))
 * @endcode
 *
 * @param manifold A 4×4 SE(3) homogeneous transformation matrix.
 * @return Six-dimensional twist vector [ωx, ωy, ωz, ρx, ρy, ρz]ᵀ.
 */
inline Vector6d manifold_to_vector(const Matrix4d& manifold)
{
    return algebra_to_vector(manifold_to_algebra(manifold));
}

// ---------------------------------------------------------------------------
// Multi-DOF forward composition
// ---------------------------------------------------------------------------

/**
 * @brief Compose a single forward-step SE(3) pose from a set of body screws
 *        and their associated velocity / displacement states.
 *
 * The DOFs are partitioned into a *base* group and an *arm* group using @p dof:
 *
 * **Base DOFs** (indices 0 … dof.base-1):
 *   The individual screw contributions are first *summed* into one aggregate
 *   twist:
 *   @code
 *     twist_base = Σ_{i=0}^{dof.base-1}  screws.row(i)ᵀ * d_state(i) * dt
 *   @endcode
 *   and then a single matrix exponential is applied:
 *   @code
 *     base_exp = vector_to_manifold(twist_base, 1.0)
 *   @endcode
 *   This models a floating base whose motion axes act simultaneously.
 *
 * **Arm DOFs** (indices dof.base … dof.total-1):
 *   Each joint is applied sequentially as its own exponential:
 *   @code
 *     arm_exp_k = vector_to_manifold(screws.row(k)ᵀ, d_state(k))
 *   @endcode
 *   This models a serial kinematic chain where joints act one after another.
 *
 * The final pose is the left-to-right product:
 * @code
 *   pose = I₄ * base_exp * arm_exp_{base} * arm_exp_{base+1} * … * arm_exp_{total-1}
 * @endcode
 *
 * @param screws  (≥ dof.total × 6) matrix.  Row i is the body screw of DOF i,
 *                expressed as [ωx, ωy, ωz, ρx, ρy, ρz].  Rows past dof.total
 *                are ignored.
 * @param d_state (≥ dof.total)-element vector of velocity / displacement
 *                values, one per DOF.  Entries past dof.total are ignored.
 * @param dof     DOF partitioning descriptor.  Must satisfy dof.total == dof.base + dof.arm.
 * @param dt      Time step (seconds).  Multiplied into base-DOF contributions only.
 * @return 4×4 SE(3) homogeneous transformation representing the composed pose.
 *
 * @throws std::invalid_argument if dof.total != dof.base + dof.arm, or if the
 *         dimensions of @p screws or @p d_state are inconsistent with @p dof.
 */
inline Matrix4d step_forward(const MatrixXd& screws,
                              const VectorXd& d_state,
                              const Dof&      dof,
                              double          dt)
{
    // --- Validate inputs -------------------------------------------------------
    if (dof.total != dof.base + dof.arm) {
        throw std::invalid_argument(
            "slithers::step_forward: dof.total must equal dof.base + dof.arm.");
    }
    // Only the first dof.total rows/entries are read, so larger inputs are
    // allowed (e.g. full screws with a base-only Dof to advance the base).
    if (screws.rows() < dof.total || screws.cols() != 6) {
        throw std::invalid_argument(
            "slithers::step_forward: screws must be (>= dof.total × 6).");
    }
    if (d_state.size() < dof.total) {
        throw std::invalid_argument(
            "slithers::step_forward: d_state length must be >= dof.total.");
    }

    // --- Base DOFs: accumulate into a single twist, then exponentiate ----------
    Vector6d screw_base = Vector6d::Zero();
    for (int i = 0; i < dof.base; ++i) {
        screw_base += screws.row(i).transpose() * d_state(i) * dt;
    }
    const Matrix4d base_exp = vector_to_manifold(screw_base, 1.0);

    // --- Arm DOFs: compose individual exponentials left-to-right --------------
    Matrix4d pose = Matrix4d::Identity() * base_exp;
    for (int i = dof.base; i < dof.total; ++i) {
        const Vector6d screw_i = screws.row(i).transpose();
        pose = pose * vector_to_manifold(screw_i, d_state(i));
    }

    return pose;
}

/**
 * @brief Compute the six-dimensional joint-space velocity / torque vector
 *        corresponding to a composed forward step.
 *
 * Computes `step_forward(screws, d_state, dof, dt)` and maps the resulting
 * SE(3) element back to ℝ⁶ via the matrix logarithm:
 * @code
 *   τ = manifold_to_vector(step_forward(screws, d_state, dof, dt))
 * @endcode
 *
 * @param screws  (dof.total × 6) body-screw matrix (same as step_forward).
 * @param d_state (dof.total)-element state vector (same as step_forward).
 * @param dof     DOF partitioning descriptor (same as step_forward).
 * @param dt      Time step in seconds (same as step_forward).
 * @return Six-dimensional vector representing the composed twist / generalised
 *         force in the Lie-algebra sense.
 *
 * @throws std::invalid_argument (propagated from step_forward) on bad inputs.
 */
inline Vector6d get_tau_joints(const MatrixXd& screws,
                                const VectorXd& d_state,
                                const Dof&      dof,
                                double          dt)
{
    return manifold_to_vector(step_forward(screws, d_state, dof, dt));
}

} // namespace slithers
