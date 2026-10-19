#include "slithers/robot_models.hpp"
#include <cmath>  // M_PI

namespace slithers {

namespace {

// Convenience alias for a 6-element column vector representing one screw axis.
using Vector6d = Eigen::Matrix<double, 6, 1>;

/// Build a screw axis from angular velocity omega and linear velocity v.
/// For revolute joints: pass omega and the joint point; compute v externally as
/// -omega.cross(point).  For prismatic joints: omega is zero and v is the
/// unit translation direction.
auto make_screw = [](const Eigen::Vector3d& omega,
                     const Eigen::Vector3d& v) -> Vector6d {
    return (Vector6d() << omega, v).finished();
};

}  // anonymous namespace

// ---------------------------------------------------------------------------
// husky_ur5e
// ---------------------------------------------------------------------------
/// Differential-drive Husky base + UR5e arm.
/// DOF layout: 2 base (rotation + forward-drive) + 6 arm = 8 total.
///
/// Base offset (metres):
///   Lbx = 0.34, Lby = 0.0, Lbz = 0.384
///
/// Screw axes are expressed in the space frame at the zero configuration.
/// Each row of `screws` is [omega(3) | v(3)].
RobotModel husky_ur5e() {
    // ---- platform geometry ------------------------------------------------
    const double Lbx = 0.34;
    const double Lby = 0.0;
    const double Lbz = 0.384;
    const Eigen::Vector3d base(Lbx, Lby, Lbz);

    // ---- screw axes -------------------------------------------------------
    // Helper: revolute screw  →  v = -omega × point
    // Helper: prismatic screw →  omega = 0, v = direction

    // S1 – base yaw (revolute about world z through origin)
    const Eigen::Vector3d w1(0, 0, 1);
    const Eigen::Vector3d p1(0, 0, 0);
    const Vector6d S1 = make_screw(w1, -w1.cross(p1));

    // S2 – base forward drive (prismatic along world x)
    const Vector6d S2 = make_screw(Eigen::Vector3d(0, 0, 0),
                                   Eigen::Vector3d(1, 0, 0));

    // S3 – arm joint 1: revolute about z through (base + [0,0,0.163])
    const Eigen::Vector3d w3(0, 0, 1);
    const Eigen::Vector3d p3 = Eigen::Vector3d(0, 0, 0.163) + base;
    const Vector6d S3 = make_screw(w3, -w3.cross(p3));

    // S4 – arm joint 2: revolute about y through (base + [0,0,0.163])
    const Eigen::Vector3d w4(0, 1, 0);
    const Eigen::Vector3d p4 = Eigen::Vector3d(0, 0, 0.163) + base;
    const Vector6d S4 = make_screw(w4, -w4.cross(p4));

    // S5 – arm joint 3: revolute about y through (base + [0.425,0,0.162])
    const Eigen::Vector3d w5(0, 1, 0);
    const Eigen::Vector3d p5 = Eigen::Vector3d(0.425, 0, 0.162) + base;
    const Vector6d S5 = make_screw(w5, -w5.cross(p5));

    // S6 – arm joint 4: revolute about y through (base + [0.817,0.133,0.162])
    const Eigen::Vector3d w6(0, 1, 0);
    const Eigen::Vector3d p6 = Eigen::Vector3d(0.817, 0.133, 0.162) + base;
    const Vector6d S6 = make_screw(w6, -w6.cross(p6));

    // S7 – arm joint 5: revolute about -z through (base + [0.817,0.133,0.063])
    const Eigen::Vector3d w7(0, 0, -1);
    const Eigen::Vector3d p7 = Eigen::Vector3d(0.817, 0.133, 0.063) + base;
    const Vector6d S7 = make_screw(w7, -w7.cross(p7));

    // S8 – arm joint 6: revolute about y through (base + [0.817,0.233,0.063])
    const Eigen::Vector3d w8(0, 1, 0);
    const Eigen::Vector3d p8 = Eigen::Vector3d(0.817, 0.233, 0.063) + base;
    const Vector6d S8 = make_screw(w8, -w8.cross(p8));

    // ---- assemble n×6 screw matrix (each row = one joint) ----------------
    MatrixXd screws(8, 6);
    screws.row(0) = S1.transpose();
    screws.row(1) = S2.transpose();
    screws.row(2) = S3.transpose();
    screws.row(3) = S4.transpose();
    screws.row(4) = S5.transpose();
    screws.row(5) = S6.transpose();
    screws.row(6) = S7.transpose();
    screws.row(7) = S8.transpose();

    // ---- zero-configuration end-effector pose (SE(3)) --------------------
    Matrix4d config_state;
    config_state <<
        -1,  0,  0,  0.817 + Lbx,
         0,  0,  1,  0.233 + Lby,
         0,  1,  0,  0.063 + Lbz,
         0,  0,  0,  1;

    // ---- joint limits -----------------------------------------------------
    VectorXd state_min(8), state_max(8);
    state_min << -M_PI, -5.0, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI;
    state_max <<  M_PI,  5.0,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI;

    return RobotModel{screws, config_state, state_min, state_max,
                      /*base_dof=*/2, /*arm_dof=*/6};
}

// ---------------------------------------------------------------------------
// husky_ur5e_holo
// ---------------------------------------------------------------------------
/// Holonomic Husky base + UR5e arm.
/// DOF layout: 3 base (yaw + x-drive + y-drive) + 6 arm = 9 total.
///
/// Same platform geometry as husky_ur5e; adds an extra prismatic y-axis.
RobotModel husky_ur5e_holo() {
    // ---- platform geometry (same as husky_ur5e) --------------------------
    const double Lbx = 0.34;
    const double Lby = 0.0;
    const double Lbz = 0.384;
    const Eigen::Vector3d base(Lbx, Lby, Lbz);

    // ---- screw axes -------------------------------------------------------
    // S1 – base yaw (revolute about z through origin)
    const Eigen::Vector3d w1(0, 0, 1);
    const Eigen::Vector3d p1(0, 0, 0);
    const Vector6d S1 = make_screw(w1, -w1.cross(p1));

    // S2 – holonomic x-drive (prismatic along x)
    const Vector6d S2 = make_screw(Eigen::Vector3d(0, 0, 0),
                                   Eigen::Vector3d(1, 0, 0));

    // S3 – holonomic y-drive (prismatic along y)
    const Vector6d S3 = make_screw(Eigen::Vector3d(0, 0, 0),
                                   Eigen::Vector3d(0, 1, 0));

    // S4 – arm joint 1: revolute about z through (base + [0,0,0.163])
    const Eigen::Vector3d w4(0, 0, 1);
    const Eigen::Vector3d p4 = Eigen::Vector3d(0, 0, 0.163) + base;
    const Vector6d S4 = make_screw(w4, -w4.cross(p4));

    // S5 – arm joint 2: revolute about y through (base + [0,0,0.163])
    const Eigen::Vector3d w5(0, 1, 0);
    const Eigen::Vector3d p5 = Eigen::Vector3d(0, 0, 0.163) + base;
    const Vector6d S5 = make_screw(w5, -w5.cross(p5));

    // S6 – arm joint 3: revolute about y through (base + [0.425,0,0.162])
    const Eigen::Vector3d w6(0, 1, 0);
    const Eigen::Vector3d p6 = Eigen::Vector3d(0.425, 0, 0.162) + base;
    const Vector6d S6 = make_screw(w6, -w6.cross(p6));

    // S7 – arm joint 4: revolute about y through (base + [0.817,0.133,0.162])
    const Eigen::Vector3d w7(0, 1, 0);
    const Eigen::Vector3d p7 = Eigen::Vector3d(0.817, 0.133, 0.162) + base;
    const Vector6d S7 = make_screw(w7, -w7.cross(p7));

    // S8 – arm joint 5: revolute about -z through (base + [0.817,0.133,0.063])
    const Eigen::Vector3d w8(0, 0, -1);
    const Eigen::Vector3d p8 = Eigen::Vector3d(0.817, 0.133, 0.063) + base;
    const Vector6d S8 = make_screw(w8, -w8.cross(p8));

    // S9 – arm joint 6: revolute about y through (base + [0.817,0.233,0.063])
    const Eigen::Vector3d w9(0, 1, 0);
    const Eigen::Vector3d p9 = Eigen::Vector3d(0.817, 0.233, 0.063) + base;
    const Vector6d S9 = make_screw(w9, -w9.cross(p9));

    // ---- assemble n×6 screw matrix ----------------------------------------
    MatrixXd screws(9, 6);
    screws.row(0) = S1.transpose();
    screws.row(1) = S2.transpose();
    screws.row(2) = S3.transpose();
    screws.row(3) = S4.transpose();
    screws.row(4) = S5.transpose();
    screws.row(5) = S6.transpose();
    screws.row(6) = S7.transpose();
    screws.row(7) = S8.transpose();
    screws.row(8) = S9.transpose();

    // ---- zero-configuration end-effector pose (same as differential model)
    Matrix4d config_state;
    config_state <<
        -1,  0,  0,  0.817 + Lbx,
         0,  0,  1,  0.233 + Lby,
         0,  1,  0,  0.063 + Lbz,
         0,  0,  0,  1;

    // ---- joint limits -----------------------------------------------------
    VectorXd state_min(9), state_max(9);
    state_min << -M_PI, -2.0, -2.0, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI;
    state_max <<  M_PI,  2.0,  2.0,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI;

    return RobotModel{screws, config_state, state_min, state_max,
                      /*base_dof=*/3, /*arm_dof=*/6};
}

// ---------------------------------------------------------------------------
// reduced_robot
// ---------------------------------------------------------------------------
/// Minimal 4-DOF robot for unit testing.
/// DOF layout: 1 base (x-drive) + 3 arm = 4 total.
///
/// Link lengths:
///   L1=0.1  (base x offset)
///   L2=0.1  (arm mount height)
///   L3=0.5  (upper arm)
///   L4=0.5  (forearm)
RobotModel reduced_robot() {
    const double L1 = 0.1;
    const double L2 = 0.1;
    const double L3 = 0.5;
    const double L4 = 0.5;
    const Eigen::Vector3d base(L1, 0, 0);

    // ---- screw axes -------------------------------------------------------
    // S1 – base drive: prismatic along x
    const Vector6d S1 = make_screw(Eigen::Vector3d(0, 0, 0),
                                   Eigen::Vector3d(1, 0, 0));

    // S2 – arm joint 1: revolute about z through (base + [0,0,0])
    const Eigen::Vector3d w2(0, 0, 1);
    const Eigen::Vector3d p2 = Eigen::Vector3d(0, 0, 0) + base;
    const Vector6d S2 = make_screw(w2, -w2.cross(p2));

    // S3 – arm joint 2: revolute about y through (base + [0,0,L2])
    const Eigen::Vector3d w3(0, 1, 0);
    const Eigen::Vector3d p3 = Eigen::Vector3d(0, 0, L2) + base;
    const Vector6d S3 = make_screw(w3, -w3.cross(p3));

    // S4 – arm joint 3: revolute about y through (base + [0,0,L2+L3])
    const Eigen::Vector3d w4(0, 1, 0);
    const Eigen::Vector3d p4 = Eigen::Vector3d(0, 0, L2 + L3) + base;
    const Vector6d S4 = make_screw(w4, -w4.cross(p4));

    // ---- assemble n×6 screw matrix ----------------------------------------
    MatrixXd screws(4, 6);
    screws.row(0) = S1.transpose();
    screws.row(1) = S2.transpose();
    screws.row(2) = S3.transpose();
    screws.row(3) = S4.transpose();

    // ---- zero-configuration end-effector pose ----------------------------
    // Tool tip is directly above the base in z by L1+L2+L3+L4.
    Matrix4d config_state;
    config_state <<
        1,  0,  0,  0,
        0,  1,  0,  0,
        0,  0,  1,  L1 + L2 + L3 + L4,
        0,  0,  0,  1;

    // ---- joint limits -----------------------------------------------------
    VectorXd state_min(4), state_max(4);
    state_min << -1.0, -2*M_PI, -2*M_PI, -2*M_PI;
    state_max <<  1.0,  2*M_PI,  2*M_PI,  2*M_PI;

    return RobotModel{screws, config_state, state_min, state_max,
                      /*base_dof=*/1, /*arm_dof=*/3};
}

}  // namespace slithers
