// =============================================================================
// slithers_demo.cpp — ROS-free example for the SLITHERS library
//
// Runs the same planning loop as the slithers_ros2 node (MATLAB main.m), but
// prints results instead of publishing them. Use this as the template for
// driving SLITHERS from a simulator (Isaac Sim, MuJoCo, ...): replace the
// printout with "send joint_velocities to the sim as joint velocity commands".
//
// Usage: slithers_demo [holo|husky]
// =============================================================================

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "slithers/slithers.hpp"

int main(int argc, char ** argv)
{
    using namespace slithers;

    const std::string robot_type = (argc > 1) ? argv[1] : "holo";
    const double time = 20.0;

    // ---- 1. Robot model + planner (MATLAB-default weights) -------------------
    const RobotModel robot = (robot_type == "husky") ? husky_ur5e() : husky_ur5e_holo();
    Planner planner(robot);
    const double dt = planner.config().dt;

    // ---- 2. Reference paths --------------------------------------------------
    const std::vector<std::pair<std::string, std::vector<Matrix4d>>> paths = {
        {"helix",            generate_helix(           0.5, 0.05, dt, time)},
        {"sine",             generate_sine(            0.0, 0.3,  dt, time)},
        {"horizontal_helix", generate_horizontal_helix(0.3, 0.15, dt, time)},
        {"spiral_sine",      generate_spiral_sine(     2.0, 0.3,  dt, time)},
    };

    std::cout << "SLITHERS demo: robot=" << robot_type
              << "  dof=" << planner.dof().total << "  dt=" << dt << "\n";

    // ---- 3. Planning loop ----------------------------------------------------
    for (const auto & [name, path] : paths) {
        planner.reset();

        double max_err = 0.0;
        double sum_err = 0.0;

        for (const Matrix4d & ee_desired : path) {
            // In a simulator: optionally feed back measured state first with
            // planner.set_base_pose(...) / planner.set_state(...), then send
            // r.joint_velocities to the robot.
            const StepResult r = planner.step(ee_desired);

            const double err =
                (r.tool_pose.block<3, 1>(0, 3) - ee_desired.block<3, 1>(0, 3)).norm();
            max_err = std::max(max_err, err);
            sum_err += err;
        }

        std::cout << "  " << name << ": steps=" << path.size()
                  << "  mean_pos_err=" << sum_err / static_cast<double>(path.size())
                  << " m  max_pos_err=" << max_err << " m\n";
    }

    return 0;
}
