#include "slithers/planner.hpp"
#include <algorithm>  // std::min
#include <stdexcept>  // std::invalid_argument

namespace slithers {

// ---------------------------------------------------------------------------
// PlannerConfig
// ---------------------------------------------------------------------------
PlannerConfig PlannerConfig::defaults(const RobotModel& robot) {
    PlannerConfig config;
    config.lambda_e = Eigen::Matrix<double, 6, 1>::Constant(25.0);

    // MATLAB main.m: base joints 1.0, first two arm joints 0.25, rest 0.1
    const int n = robot.base_dof + robot.arm_dof;
    config.lambda_v = Eigen::VectorXd::Constant(n, 0.1);
    config.lambda_v.head(robot.base_dof).setConstant(1.0);
    for (int i = robot.base_dof; i < std::min(robot.base_dof + 2, n); ++i) {
        config.lambda_v(i) = 0.25;
    }
    return config;
}

// ---------------------------------------------------------------------------
// Planner
// ---------------------------------------------------------------------------
Planner::Planner(const RobotModel& robot)
    : Planner(robot, PlannerConfig::defaults(robot)) {}

Planner::Planner(const RobotModel& robot, const PlannerConfig& config)
    : robot_(robot), config_(config) {
    dof_.base  = robot.base_dof;
    dof_.arm   = robot.arm_dof;
    dof_.total = robot.base_dof + robot.arm_dof;

    dof_base_.base  = robot.base_dof;
    dof_base_.arm   = 0;
    dof_base_.total = robot.base_dof;

    if (config.lambda_v.size() != dof_.total) {
        throw std::invalid_argument(
            "slithers::Planner: config.lambda_v length must equal the robot's total DOF.");
    }
    if (config.dt <= 0.0) {
        throw std::invalid_argument("slithers::Planner: config.dt must be positive.");
    }

    params_.screws       = robot.screws;
    params_.config_state = robot.config_state;
    params_.state_min    = robot.state_min;
    params_.state_max    = robot.state_max;
    params_.lambda_e     = config.lambda_e;
    params_.lambda_j     = config.lambda_j;
    params_.lambda_v     = config.lambda_v;
    params_.dt           = config.dt;
    params_.dof          = dof_;

    reset();
}

void Planner::reset() {
    x_         = Eigen::VectorXd::Zero(dof_.total);
    dx_        = Eigen::VectorXd::Zero(dof_.total);
    base_pose_ = Eigen::Matrix4d::Identity();
}

void Planner::set_state(const Eigen::VectorXd& state) {
    if (state.size() != dof_.total) {
        throw std::invalid_argument(
            "slithers::Planner::set_state: state length must equal the robot's total DOF.");
    }
    x_ = state;
}

StepResult Planner::step(const Eigen::Matrix4d& ee_goal) {
    // 1. Advance the base with the previous command (base DOFs only)
    base_pose_ = base_pose_ * step_forward(robot_.screws, dx_, dof_base_, config_.dt);
    params_.base_pose = base_pose_;

    // 2. Solve for the new joint velocity command
    dx_ = ik_optimization(x_, dx_, ee_goal, params_);

    // 3. Tool pose reached with that command
    StepResult result;
    result.joint_velocities = dx_;
    result.base_pose        = base_pose_;
    result.tool_pose        = base_pose_
                            * step_forward(robot_.screws, dx_, dof_, config_.dt)
                            * robot_.config_state;

    // 4. Roll state forward: base joints reset, arm state takes dx
    x_.head(dof_.base).setZero();
    x_.tail(dof_.arm) = dx_.tail(dof_.arm);

    return result;
}

}  // namespace slithers
