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
