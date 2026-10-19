#pragma once

/**
 * @file planner.hpp
 * @brief Stateful step-by-step planner for the SLITHERS library.
 *
 * Wraps the per-time-step planning loop (MATLAB lie_theory/main.m) so that
 * simulators and ROS nodes share one implementation:
 *
 * @code
 *   slithers::Planner planner(slithers::husky_ur5e_holo());
 *   for (const auto& goal : path) {
 *       const slithers::StepResult r = planner.step(goal);
 *       send_to_robot(r.joint_velocities);   // sim / hardware command
 *   }
 * @endcode
 *
 * Each step():
 *   1. advances the base pose using the previous joint velocity command
 *      (base DOFs only),
 *   2. solves the IK for the new joint velocity command dx,
 *   3. computes the resulting tool pose,
 *   4. rolls the joint state forward (base joints reset, arm joints take dx).
 *
 * Dependencies:
 *   - Eigen 3 (Eigen/Dense)
 *   - slithers/robot_models.hpp, slithers/ik_solver.hpp
 *   - C++17
 */

#include <Eigen/Dense>
#include "slithers/robot_models.hpp"
#include "slithers/ik_solver.hpp"

namespace slithers {

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Tunable planner settings (time step and IK weights).
struct PlannerConfig {
    double                       dt{0.1};        ///< Time step [s]
    Eigen::Matrix<double, 6, 1>  lambda_e;       ///< Cartesian error weights (6)
    double                       lambda_j{0.001};///< Jerk penalty weight (reserved for future use)
    Eigen::VectorXd              lambda_v;       ///< Joint velocity weights (n)

    /**
     * @brief Default settings matching MATLAB main.m for @p robot.
     *
     * lambda_e = 25 for every axis; lambda_v = 1.0 for base joints,
     * 0.25 for the first two arm joints and 0.1 for the remaining arm joints.
     */
    static PlannerConfig defaults(const RobotModel& robot);
};

// ---------------------------------------------------------------------------
// Step output
// ---------------------------------------------------------------------------

/// Output of one Planner::step().
struct StepResult {
    Eigen::VectorXd joint_velocities;  ///< IK command dx (n): send this to the robot
    Eigen::Matrix4d tool_pose;         ///< End-effector SE(3) pose reached with dx (world frame)
    Eigen::Matrix4d base_pose;         ///< Base SE(3) pose used for this step (world frame)
};

// ---------------------------------------------------------------------------
// Planner
// ---------------------------------------------------------------------------

class Planner {
public:
    /// Construct with MATLAB-default settings (PlannerConfig::defaults).
    explicit Planner(const RobotModel& robot);

    /**
     * @brief Construct with explicit settings.
     * @throws std::invalid_argument if config.lambda_v does not match the
     *         robot's total DOF or config.dt is not positive.
     */
    Planner(const RobotModel& robot, const PlannerConfig& config);

    /// Reset to the start of a trial: zero state / velocity, identity base pose.
    void reset();

    /**
     * @brief Advance one time step toward @p ee_goal.
     * @param ee_goal  Desired end-effector SE(3) pose (world frame).
     * @return         Joint velocity command, resulting tool pose and base pose.
     */
    StepResult step(const Eigen::Matrix4d& ee_goal);

    // ---- State feedback (e.g. from a simulator) ----------------------------
    // Optional: overwrite the planner's internal estimates with measured values
    // before calling step().

    /// Set the base SE(3) pose (world frame).
    void set_base_pose(const Eigen::Matrix4d& base_pose) { base_pose_ = base_pose; }

    /// Set the joint state (n), used as the IK warm start.
    void set_state(const Eigen::VectorXd& state);

    // ---- Accessors ---------------------------------------------------------
    const RobotModel&      robot()     const { return robot_; }
    const PlannerConfig&   config()    const { return config_; }
    const Dof&             dof()       const { return dof_; }
    const Eigen::VectorXd& state()     const { return x_; }
    const Eigen::VectorXd& velocity()  const { return dx_; }
    const Eigen::Matrix4d& base_pose() const { return base_pose_; }

private:
    RobotModel      robot_;
    PlannerConfig   config_;
    Dof             dof_;       ///< Full DOF breakdown (base + arm)
    Dof             dof_base_;  ///< Base-only DOF breakdown, for advancing the base
    IKParameters    params_;

    Eigen::VectorXd x_;         ///< Joint state (IK warm start)
    Eigen::VectorXd dx_;        ///< Last joint velocity command
    Eigen::Matrix4d base_pose_;
};

}  // namespace slithers
