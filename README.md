# jetank_moveit_config

MoveIt 2 configuration for the JeTank 4‑DOF arm.

- **Planning group:** `arm` — chain `base_link → S5_link`, joints
  `S1_joint, S2_joint, S3_joint, S5_joint` (S4 is fixed; servo 4 drives the
  gripper). SRDF: `config/jetank.srdf`.
- **Execution:** `move_group` plans and executes via
  `/arm_controller/follow_joint_trajectory` (a `FollowJointTrajectory` action).
  Controllers come from Gazebo's `ign_ros2_control` (sim) or the real hardware
  interface — this package does **not** start its own `ros2_control_node` in the
  sim path.

## Launch files

| File | Starts Gazebo? | Use |
|---|---|---|
| `moveit_sim.launch.py` | **yes** (`gazebo.launch.py`, arm active) | MoveIt on top of Gazebo |
| `moveit_bringup.launch.py` | no | MoveIt + own controller mgmt (mock/serial) |
| `demo.launch.py` | no | standalone MoveIt demo (mock hardware) |

### ⚠️ `moveit_sim.launch.py` is headless by default

```bash
# Default = headless:=true, use_rviz:=false  → NO Gazebo GUI, NO RViz.
# To SEE the simulation you MUST override both:
ros2 launch jetank_moveit_config moveit_sim.launch.py headless:=false use_rviz:=true
```

Verified: with the GUI on, `move_group` comes up, `arm_controller` is active,
and a trajectory goal to `/arm_controller/follow_joint_trajectory` executes
(`SUCCEEDED`) and moves the arm in sim.

> The default RViz here is the generic MoveIt config. For the JeTank-specific
> view, use `jetank_ros_main/rviz/unified.rviz` (via `rviz.launch.py`).

### `start_gazebo:=false` — attach to an existing sim

```bash
# Run move_group only, against a Gazebo already started elsewhere
ros2 launch jetank_moveit_config moveit_sim.launch.py start_gazebo:=false
```

This is how `jetank_ros_main/sim_demo.launch.py arm:=true` folds the arm into
the unified sim without spawning a second Gazebo.

---

## ROS 2 API

`jetank_moveit_config` is a **MoveIt 2 configuration package** (build type `ament_cmake`). It defines **no runtime nodes of its own** — there is no `src/`, `include/`, `msg/`, `srv/`, or `action/` in this package. Instead it provides SRDF/config files and launch files that start **standard, third-party executables** (MoveIt `move_group`, `ros2_control` controller manager + spawners, `robot_state_publisher`, RViz). The ROS 2 interfaces below are therefore the interfaces those standard executables expose when launched with this package's configuration.

### What this package provides

| Type | File |
|---|---|
| SRDF (planning groups, named poses, collision rules) | `config/jetank.srdf` |
| Kinematics solver config | `config/kinematics.yaml` |
| Joint limits | `config/joint_limits.yaml` |
| OMPL planning pipeline | `config/ompl_planning.yaml` |
| Pilz cartesian limits | `config/pilz_cartesian_limits.yaml` |
| MoveIt → ros2_control controller mapping | `config/moveit_controllers.yaml` |
| Launch entrypoints | `launch/demo.launch.py`, `launch/moveit_bringup.launch.py`, `launch/moveit_sim.launch.py` |

The robot description is **not** owned here — it is built from `jetank_description/urdf/jetank_ros2_control.urdf.xacro`.

### Planning groups (SRDF)

| Group | Definition | Named states |
|---|---|---|
| `arm` | kinematic chain `base_link` → `S5_link` (joints `S1_joint, S2_joint, S3_joint, S5_joint`) | `home`, `ready`, `vertical`, `grasp_pre`, `grasp_reach` |
| `gripper` | `gripper_left_joint` (`gripper_right_joint` is a mimic) | `open`, `closed`, `half_open` |

End effector `gripper_ee`: parent link `S5_link`, group `gripper`. Virtual joint `virtual_joint` (fixed) attaches `base_footprint` to `world`.

### Nodes launched

| Node name | Executable (package) | Launched by | Role |
|---|---|---|---|
| `move_group` | `move_group` (`moveit_ros_move_group`) | all three | MoveIt motion planning/execution server |
| `controller_manager` | `ros2_control_node` (`controller_manager`) | `moveit_bringup` only | ros2_control manager (mock/serial). **Not** started on the sim path |
| `joint_state_broadcaster_spawner`, `arm_controller_spawner`, `gripper_controller_spawner` | `spawner` (`controller_manager`) | `moveit_bringup` only | Activate the controllers |
| `robot_state_publisher` | `robot_state_publisher` | `demo` only | `/robot_description` + TF from xacro |
| `virtual_joint_broadcaster` | `static_transform_publisher` (`tf2_ros`) | `demo` only | Static TF `world` → `base_footprint` |
| `rviz2` | `rviz2` | optional (`use_rviz`) | MoveIt motion-planning RViz plugin |

On the **sim path** (`moveit_sim.launch.py`) only `move_group` (+ optional RViz) is started; Gazebo's `ign_ros2_control` provides the controller manager, `joint_state_broadcaster`, and `arm_controller`/`gripper_controller`.

### Actions (trajectory/gripper execution)

`move_group` is the **action client**; the controllers (from Gazebo `ign_ros2_control` in sim, or the launched `ros2_control_node` otherwise) are the **servers**. Names from `config/moveit_controllers.yaml`.

| Action | Type | move_group role |
|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | client |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | client |

(`move_group` also exposes the standard MoveIt action/service set — e.g. `/move_action`, `/execute_trajectory`, planning-scene services — but those come from the upstream `move_group` executable, not from this package.)

### Key launch arguments

| Argument | Launch file(s) | Default | Meaning |
|---|---|---|---|
| `hardware` | `demo`, `moveit_bringup` | `mock` | ros2_control backend baked into the URDF: `mock` (software-only) \| `serial` (real servos) |
| `use_sim_time` | `demo`, `moveit_bringup` | `false` | Use Gazebo clock |
| `use_rviz` | all | `false` (`true` in `demo`) | Launch RViz with the MoveIt plugin |
| `headless` | `moveit_sim` | `true` | Run Gazebo server-only (no GUI) |
| `start_gazebo` | `moveit_sim` | `true` | If `false`, run `move_group` only against an already-running Gazebo |
| `rviz_config` | `moveit_sim` | `moveit_ros_visualization/launch/moveit.rviz` | RViz config to load |

### Solver / pipeline

- IK: `kdl_kinematics_plugin/KDLKinematicsPlugin` for the `arm` group (`config/kinematics.yaml`).
- Planning pipeline: `ompl` (`config/ompl_planning.yaml`).
