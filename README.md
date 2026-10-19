# slithers

**SLITHERS** — a Lie theory-based inverse-kinematics planner for mobile manipulators,
as a standalone C++17 library. It is a port of the MATLAB `lie_theory/main.m` workflow.

The library depends only on **Eigen 3**. It does not depend on ROS, so you can use it from
simulators (Isaac Sim, MuJoCo, ...), from plain C++ programs, and from ROS 2 through the
[`slithers_ros2`](../slithers_ros2) wrapper package.

```
slithers_lib/
├── include/slithers/
│   ├── slithers.hpp          umbrella header — include this
│   ├── lie_utils.hpp         SE(3)/se(3) math, step_forward, get_tau_joints (header-only)
│   ├── robot_models.hpp      RobotModel + built-in robots
│   ├── path_generators.hpp   reference end-effector paths
│   ├── ik_solver.hpp         IKParameters + ik_optimization (one IK solve)
│   └── planner.hpp           Planner — the per-time-step planning loop
├── src/                      implementations of the above
├── examples/slithers_demo.cpp  ROS-free demo / simulator template
└── cmake/slithersConfig.cmake.in  find_package(slithers) support
```

---

## Building

### Plain CMake (no ROS)

```bash
cmake -S slithers_lib -B build -DCMAKE_BUILD_TYPE=Release -DSLITHERS_BUILD_EXAMPLES=ON
cmake --build build -j
./build/slithers_demo holo         # or: husky
cmake --install build --prefix ~/.local   # optional: install for other projects
```

| CMake option | Default | Meaning |
|---|---|---|
| `SLITHERS_BUILD_EXAMPLES` | `OFF` | Build `slithers_demo` |

**Always build with `-DCMAKE_BUILD_TYPE=Release`.** The IK relies heavily on Eigen
matrix exponentials and logarithms, and unoptimised builds are many times slower.

### Using it from another CMake project

```cmake
find_package(slithers REQUIRED)        # set CMAKE_PREFIX_PATH if installed somewhere custom
target_link_libraries(my_sim_app PRIVATE slithers::slithers)
```

```cpp
#include "slithers/slithers.hpp"
```

Eigen comes in through the `slithers::slithers` target, so you don't need to find it separately.

### As a ROS 2 package (colcon)

`package.xml` declares the package name `slithers`. When `ament_cmake` is available
(under colcon) the library also exports itself as an ament package; when it isn't,
the ament steps are skipped. See `slithers_ros2/README.md` for workspace setup.

---

## How it works

### 1. Robot model — screws in the space frame

A robot is described by its **screw axes** in the space frame at the zero configuration
(product-of-exponentials form). `RobotModel` holds:

| Field | Meaning |
|---|---|
| `screws` | n×6 matrix, one row per joint: `[ωx ωy ωz  vx vy vz]` |
| `config_state` | 4×4 SE(3) end-effector pose at the zero configuration (often called *M*) |
| `state_min`, `state_max` | per-joint box limits (n) |
| `base_dof`, `arm_dof` | how many of the n joints belong to the base vs. the arm |

The **base joints always come first** (rows `0 … base_dof-1`), then the arm joints.
For a revolute joint `v = −ω × p` (p = a point on the axis); for a prismatic joint
`ω = 0` and `v` is the unit direction.

Built-in models (`robot_models.hpp`):

| Function | Base | Arm | n | Base limits |
|---|---|---|---|---|
| `husky_ur5e()` | differential: yaw (rev. z), forward drive (prism. x) | UR5e (6R) | 8 | yaw ±π, drive ±5 |
| `husky_ur5e_holo()` | holonomic: yaw, x-drive, y-drive | UR5e (6R) | 9 | yaw ±π, x/y ±2 |
| `reduced_robot()` | x-drive | 3R planar-ish test arm | 4 | ±1 |

The UR5e is mounted at `(0.34, 0, 0.384)` m on the Husky. Arm joint limits are ±2π.

To add your own robot, fill a `RobotModel` the same way (see `src/robot_models.cpp`).

### 2. Forward step on the manifold — `step_forward`

`step_forward(screws, d_state, dof, dt)` composes one SE(3) transform from a joint vector,
and treats base and arm joints differently:

```
base:  twist_base = Σ_{i<base}  S_i · d_state(i) · dt        (one exp of the summed twist)
arm:   T_arm      = Π_{i≥base}  exp([S_i] · d_state(i))       (serial chain, no dt)

pose = exp([twist_base]) · T_arm
```

- **Base entries are velocities.** They are multiplied by `dt` and act together, which
  models a mobile base moving for one time step.
- **Arm entries are joint displacements (angles).** They are *not* multiplied by `dt`,
  and the joints are composed in order like a normal serial chain.

`get_tau_joints(...)` is `manifold_to_vector(step_forward(...))`: the resulting motion
as a 6-vector twist `[ω; ρ]` (via the matrix log).

`lie_utils.hpp` also provides the conversions between twist vectors, se(3) matrices and
SE(3) matrices: `vector_to_algebra` / `algebra_to_vector`, `algebra_to_manifold` (exp) /
`manifold_to_algebra` (log), and `vector_to_manifold` / `manifold_to_vector`.

### 3. One IK solve — `ik_optimization`

For a goal pose `T_goal` (world frame) and the current base pose `T_base`, the target
is expressed in the base frame and with the zero-config offset removed:

```
tau_goal = log( T_base⁻¹ · T_goal · M⁻¹ )            (6-vector)
```

The solver then finds the joint vector `x` that minimises

```
J(x) = ‖ λe ∘ (get_tau_joints(x) − tau_goal) ‖₂  +  ‖ λv ∘ x ‖₂
          pose-tracking error                       joint-effort regularisation
```

subject to `state_min ≤ x ≤ state_max`, using:

- warm start from `current_state` (clamped to the limits),
- central finite-difference gradient (ε = 1e-6),
- backtracking line search over α ∈ {1, 0.5, 0.1, 0.01, 0.001}, clamping each candidate
  to the limits; a step is accepted only if it lowers J,
- at most 200 iterations; stops early when ‖∇J‖₂ < 1e-6 or no step size improves J.

Weights live in `IKParameters`:

| Field | Meaning |
|---|---|
| `lambda_e` (6) | per-axis weight on the twist error `[ωx ωy ωz ρx ρy ρz]` |
| `lambda_v` (n) | per-joint penalty on the solution; larger → that joint is used less |
| `lambda_j` | jerk weight — **reserved, not used yet** |
| `dt` | time step [s] |

The cost uses plain norms, not squared ones, to match the MATLAB reference. With squared
norms and λe = 25 the gradients get large enough that the fixed step sizes push the
solution straight to the joint limits.

### 4. The planning loop — `Planner`

`Planner` runs the per-time-step loop from MATLAB `main.m`, so simulators and the ROS
node don't each need their own copy of it. It keeps track of the joint state `x`,
the last command `dx` and the base pose. Each `step(T_goal)`:

1. **Advances the base** with the previous command, base joints only:
   `T_base ← T_base · step_forward(screws, dx, base-only Dof, dt)`
2. **Solves the IK:** `dx ← ik_optimization(x, dx, T_goal, params)`
3. **Computes the tool pose** reached: `T_tool = T_base · step_forward(screws, dx, dof, dt) · M`
4. **Rolls the state forward:** base entries of `x` reset to 0, arm entries of `x` ← arm entries of `dx`

and returns a `StepResult { joint_velocities (= dx), tool_pose, base_pose }`.

```cpp
#include "slithers/slithers.hpp"

slithers::Planner planner(slithers::husky_ur5e_holo());     // MATLAB-default weights
auto path = slithers::generate_helix(0.5, 0.05, planner.config().dt, 20.0);

for (const auto& goal : path) {
    slithers::StepResult r = planner.step(goal);
    // r.joint_velocities -> robot / simulator
    // r.tool_pose, r.base_pose -> logging / visualisation
}
```

**Configuration.** `PlannerConfig::defaults(robot)` gives the MATLAB settings: `dt = 0.1`,
`λe = 25` on every axis, `λv = 1.0` for base joints, `0.25` for the first two arm joints and
`0.1` for the rest. To change them, pass your own config:

```cpp
slithers::PlannerConfig cfg = slithers::PlannerConfig::defaults(robot);
cfg.dt = 0.02;
cfg.lambda_v(0) = 2.0;          // use base yaw less
slithers::Planner planner(robot, cfg);   // throws if lambda_v.size() != n or dt <= 0
```

**Methods**

| Method | Purpose |
|---|---|
| `step(T_goal)` | advance one time step (above) |
| `reset()` | start a new trial: `x = dx = 0`, `T_base = I` |
| `set_base_pose(T)` | overwrite the base pose estimate (e.g. with the simulator's base pose) |
| `set_state(x)` | overwrite the joint state / IK warm start (n) |
| `robot()`, `config()`, `dof()`, `state()`, `velocity()`, `base_pose()` | read-only access |

### 5. Reference paths — `path_generators.hpp`

Each generator returns `floor(time/dt) + 1` SE(3) poses sampled at `t = 0, dt, …, time`.
The arguments are `(a, b, dt, time)`; the values in brackets are the ones `main.m` uses.

| Function | Position (x, y, z) | Orientation |
|---|---|---|
| `generate_helix` [a=0.5, b=0.05] | `a·cos t, a·sin t, b·t + 0.5` | yaw t, then axis permutation |
| `generate_sine` [b=0.3] | `t + 2, 0.447, b·sin(πt/2) + 0.5` | fixed (`a` unused) |
| `generate_horizontal_helix` [a=0.3, b=0.15] | `b·t + 1, a·sin t + 0.5, 0.5a·cos t + a + 0.25` | fixed |
| `generate_spiral_sine` [a=2, b=0.3] | `t + sin(at), cos(at), b·sin t + 3b` | fixed, yaw −45° |

You can feed any `std::vector<Eigen::Matrix4d>` (or one pose at a time) to the planner;
these generators are only the test trajectories from the paper / MATLAB code.

---

## Using SLITHERS with a simulator

`examples/slithers_demo.cpp` is the template. The general pattern, independent of the
simulator:

```cpp
slithers::Planner planner(robot);

while (sim_running) {
    // (optional) feed back measured state so the planner doesn't drift from the sim
    planner.set_base_pose(base_pose_from_sim());   // 4x4, world frame
    // planner.set_state(...);                      // n-vector, same layout as below

    slithers::StepResult r = planner.step(next_goal_pose());

    // r.joint_velocities layout: [ base_0 … base_{b-1} | arm_0 … arm_{a-1} ]
    apply_base_command(r.joint_velocities.head(robot.base_dof));   // velocities (rad/s, m/s)
    apply_arm_command (r.joint_velocities.tail(robot.arm_dof));    // see note below

    sim_step(planner.config().dt);   // advance the sim by the planner's dt
}
```

Things to get right when connecting a simulator:

- **Joint order.** The output follows the row order of `robot.screws`: base joints first,
  then arm joints from shoulder to wrist. Map these to your sim's actuators explicitly.
  Don't assume the sim's joint order is the same.
- **What the arm entries mean.** By the `step_forward` convention above, base entries are
  velocities, but **arm entries are joint angles relative to the zero configuration**. The
  tool pose the planner reports is computed that way. So in a sim, drive the arm with
  *position* targets equal to these values (or turn them into velocities yourself), and drive
  the base with velocity commands. The ROS topic is still called `joint_velocities` for
  compatibility with the MATLAB port.
- **Zero configuration.** `config_state` and the screws assume the UR5e zero pose used in
  `robot_models.cpp`. Check that your sim model's joint zeros match, or add offsets.
- **Frames.** Goals and the returned poses are in the world frame. The base starts at the
  identity unless you call `set_base_pose`.
- **Time step.** Advance the simulation by `config.dt` for each `step()`, or set `cfg.dt`
  to the sim's control period.

---

## Performance

Per-step cost is up to 200 iterations × 2n finite-difference cost evaluations, each with
a matrix exp/log. Measured on this machine with a Release build, 201-step paths:

| Robot | All four demo paths |
|---|---|
| `husky_ur5e_holo` | ≈ 0.8 s |
| `husky_ur5e` | ≈ 13 s (the differential base needs many more iterations) |

So `holo` is comfortably real-time at `dt = 0.1`. `husky` averages around 15 ms per step,
with some steps much slower.

## Known limitations

- `lambda_j` (jerk penalty) is declared but not used yet.
- Tracking error on the demo paths with the default weights is about 0.1–0.5 m on average;
  `spiral_sine` with `husky_ur5e` diverges (≈ 7 m average error). This matches the original
  `slithers_ros2` implementation and is worth checking against the MATLAB results.
- The gradient is computed by finite differences. An analytic Jacobian would be much faster.

## License

MIT — Siddharth Kumar (sks4zk@virginia.edu)
