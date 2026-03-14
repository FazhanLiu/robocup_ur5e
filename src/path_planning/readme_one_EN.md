## one_shot_planner (One-Shot Planning)

This module provides a **one-shot full trajectory planning** API on top of the `path_planning` ACO + RRT* planner: it plans from start to goal, converts the path to `JointTrajectory`, and sends it via `/motion/command` for `motion_control` to execute.

**Aligned with `ur5e_teach_pendant_ui`**: It does not call IK/FK directly; it relies on `motion_control_node`. Current end-effector pose is obtained from **TF**, current joints from **`/joint_states`**, and goal joints from **config / upper layer**. Interaction with motion_control is only through **`MotionCommand`**.

**Collision checking**: Besides testing the 7 link-frame origins against obstacles along the path, RRT* supports **end-effector bounding box** at the current pose (see `ee_half_extents` / `~end_effector_collision_box`), so the end-effector is kept clear of obstacles along the whole path.

---

### 1. Related files

- **`nodes/one_shot_planner.py`**  
  Core one-shot planning functions for use by other nodes or logic. Exposes `plan_one_shot` (goal as xyz + goal_joints) and `plan_one_shot_from_goal_pose` (goal as 4×4 pose matrix; IK is called internally).
- **`nodes/demo_one_shot_planning.py`**  
  Demo node: gets current pose from TF, current joints from `/joint_states`, goal joints from parameters, then plans and publishes `EXECUTE_TRAJECTORY` to `/motion/command`.

---

### 2. Core API of one_shot_planner

#### 2.1 `get_default_virtual_grasp_point()`

- Returns the default virtual grasp point `[x, y, z]` in `base_link` frame, consistent with `virtual_grasp_point` in `config/planning_config.yaml` (default `[0.35, 0.0, 0.2]`, table center at table-top height). Typically used as the ACO goal Cartesian point.

#### 2.2 `plan_one_shot(...)`

**Function signature:**

```python
path_joints, trajectory = plan_one_shot(
    start_xyz,      # [x, y, z] start Cartesian position, e.g. from TF (same as teach pendant)
    start_joints,   # start joint angles (6,), from /joint_states
    goal_xyz,       # [x, y, z] goal Cartesian position for ACO, usually virtual_grasp_point
    goal_joints,    # goal joint angles (6,), from config/upper layer; no IK here
    obstacles=None,
    bounds=None,
    frame_id="base_link",
    aco_grid_res=0.08,
    aco_n_ants=40,
    aco_n_iters=30,
    aco_greedy_prob=0.3,
    aco_elite_deposit_ratio=2.0,
    rrt_step_size=0.12,
    rrt_max_iter=4000,
    return_vis_data=False,
    ee_half_extents=None,  # EE-frame box half-lengths (hx, hy, hz); None => read ~end_effector_collision_box
)
```

- **`start_xyz`** (required)  
  - Start Cartesian position `[x, y, z]` in `base_link`.  
  - Same as teach pendant: from **TF** `base_link` → `tcp_link` (e.g. `gripper_tip_link`).

- **`start_joints`** (required)  
  - Start joint angles (6-DOF), same order as `motion_control`.  
  - From **`/joint_states`** subscription.

- **`goal_xyz`** (required)  
  - Goal Cartesian position `[x, y, z]` for ACO path; usually `virtual_grasp_point` or `get_default_virtual_grasp_point()`.

- **`goal_joints`** (required)  
  - Goal joint angles (6-DOF).  
  - **No IK here**; provided by config (e.g. `~goal_joints`) or upper logic; motion_control is trusted for the target pose.

- **`obstacles`** (optional)  
  - Format: `[((cx, cy, cz), (hx, hy, hz)), ...]`.  
  - `None` uses `GAZEBO_DEFAULT_OBSTACLES` plus ground.

- **`ee_half_extents`** (optional)  
  - End-effector bounding box half-lengths `(hx, hy, hz)` (m) in **EE frame**, used for pose/geometry collision along the path; RRT* checks whether the 8 corners of this box (at the current end-effector pose) intersect obstacles.  
  - `None` reads ROS param `~end_effector_collision_box` (e.g. `end_effector_collision_box: [0.02, 0.02, 0.05]` in `planning_config.yaml`); use `(0, 0, 0)` to disable EE box and keep only the 7 link-point checks.

- **Returns**  
  - `path_joints`: RRT* joint path, or `None` on failure.  
  - `trajectory`: `trajectory_msgs/JointTrajectory`, can be wrapped as `MotionCommand(EXECUTE_TRAJECTORY)` and sent to `/motion/command`.

**Config**: In `config/planning_config.yaml`, `end_effector_collision_box: [0.02, 0.02, 0.05]` is the default EE box half-lengths (~4×4×10 cm). Adjust for your gripper/EE size, or set to `[0, 0, 0]` to disable EE geometry checking.

#### 2.2.1 `plan_one_shot_from_goal_pose(goal_pose_4x4, start_xyz, start_joints, ...)`

Wrapper that takes a **goal 4×4 Cartesian matrix (position + rotation)**: it converts the matrix to a pose, calls `/motion/compute_ik` to get `goal_joints`, then calls `plan_one_shot`. Use when the upper layer only provides a target end-effector pose, not joint angles.

**Signature outline:**

```python
path_joints, trajectory = plan_one_shot_from_goal_pose(
    goal_pose_4x4,     # 4x4 homogeneous transform (numpy or list), target EE pose in base frame
    start_xyz,         # [x, y, z] start Cartesian position
    start_joints,      # start joint angles (6,)
    obstacles=None,
    bounds=None,
    frame_id="base_link",
    # ... other ACO/RRT args same as plan_one_shot ...
    return_vis_data=False,
    ee_half_extents=None,
    seed_joints_for_ik=None,  # optional IK seed; None => use start_joints
    ik_timeout=2.0,           # timeout (s) for /motion/compute_ik
)
```

- **`goal_pose_4x4`** (required): 4×4 homogeneous transform; `T[0:3, 3]` is position, `T[0:3, 0:3]` is rotation; converted to `PoseStamped` and passed to IK.
- **`seed_joints_for_ik`** (optional): IK initial guess; default is `start_joints`.
- **Returns**: Same as `plan_one_shot`; on IK failure or invalid matrix, returns `(None, None)` (or `(None, None, None)` when `return_vis_data=True`).

**Dependency**: Requires `motion_control` to provide `/motion/compute_ik` (request: `PoseStamped` with position and quaternion orientation).

**Example** — goal position `[0.35, 0, 0.2]`, identity rotation:

```python
import numpy as np
from one_shot_planner import plan_one_shot_from_goal_pose, build_motion_command_execute_trajectory

goal_4x4 = np.eye(4)
goal_4x4[0:3, 3] = [0.35, 0.0, 0.2]   # goal point; set goal_4x4[:3, :3] for arbitrary orientation

path_joints, traj = plan_one_shot_from_goal_pose(
    goal_pose_4x4=goal_4x4,
    start_xyz=[0.2, 0.0, 0.3],
    start_joints=[0, -1.57, 0, -1.57, 0, 0],
)
if path_joints and traj:
    cmd = build_motion_command_execute_trajectory(traj)
    pub.publish(cmd)
```

#### 2.3 `joints_to_trajectory(path_joints, joint_names, time_step=0.5)`

- Converts a joint path to `JointTrajectory`; `time_step` is the time (seconds) between consecutive points.

#### 2.4 `build_motion_command_execute_trajectory(trajectory)`

- Builds `MotionCommand` with `command_type=EXECUTE_TRAJECTORY`, `max_velocity=1.0`, `max_acceleration=1.0`, same as the teach pendant; can be published directly to `/motion/command`.

---

### 3. demo_one_shot_planning

`nodes/demo_one_shot_planning.py` is a runnable demo with the same logic as the teach pendant: TF + `/joint_states` + params; it does not call IK/FK.

#### 3.1 Prerequisites

- `roscore` running.  
- `motion_control_node` running (subscribes to `/motion/command`).  
- TF has `base_link` → `gripper_tip_link` (or `~tcp_link` if set).  
- Goal joints configured via `~goal_joints` or default `~goal_joints_default`.  
- Optional: Gazebo/RViz, `/joint_states`, and point-cloud topic for full verification.

#### 3.2 Running the demo

```bash
cd ~/robocup_ur5e
source devel/setup.bash
rosrun path_planning demo_one_shot_planning.py
```

With parameters:

```bash
# Set goal joints (one of param or default required)
rosrun path_planning demo_one_shot_planning.py _goal_joints:="[0, -2.0, 1.2, -1.57, -1.57, 0]"

# Set virtual grasp point (used as ACO goal_xyz)
rosrun path_planning demo_one_shot_planning.py _virtual_grasp_point:="[0.35, 0.0, 0.2]"

# Set EE collision box half-lengths (EE frame [hx, hy, hz] m); [0,0,0] disables EE geometry check
rosrun path_planning demo_one_shot_planning.py _end_effector_collision_box:="[0.02, 0.02, 0.05]"
```

#### 3.3 Demo flow

1. **Start pose**  
   - Same as teach pendant: **TF** lookup `base_link` → `tcp_link` for current EE position `start_xyz`; on failure uses `~start_xyz`.

2. **Start joints**  
   - One-shot read from **`/joint_states`** as `start_joints`; on timeout uses `~home_joints`.

3. **Goal**  
   - `goal_xyz`: from `~virtual_grasp_point` or `get_default_virtual_grasp_point()`.  
   - `goal_joints`: from `~goal_joints`; if unset, `~goal_joints_default` (no IK in demo).

4. **Obstacles**  
   - If point-cloud topic (e.g. `/camera/depth/points`) has data, use `pointcloud_to_obstacles`; otherwise `GAZEBO_DEFAULT_OBSTACLES`.

5. **Plan and send**  
   - Call `plan_one_shot(start_xyz, start_joints, goal_xyz, goal_joints, obstacles)`; EE collision box from `~end_effector_collision_box` (enabled by default).  
   - Build `MotionCommand` with `build_motion_command_execute_trajectory(trajectory)` and publish to **`/motion/command`** for motion_control to execute.

---

### 4. Reusing one-shot planning in other nodes

Same pattern as the teach pendant: no direct IK/FK; use TF, `/joint_states`, and config, and talk to motion_control via `MotionCommand`.

1. Get current EE position `start_xyz` from **TF** (e.g. `base_link` → `gripper_tip_link`).  
2. Get current joints `start_joints` from **`/joint_states`**.  
3. Choose goal: if you have joint angles + Cartesian point, call `plan_one_shot(...)`; if you have a **4×4 goal pose matrix**, call `plan_one_shot_from_goal_pose(goal_pose_4x4, ...)` (IK is called inside).  
4. Publish the returned trajectory to `/motion/command` with `build_motion_command_execute_trajectory`.

Example (pseudo-code):

```python
import tf2_ros
from one_shot_planner import (
    plan_one_shot,
    get_default_virtual_grasp_point,
    build_motion_command_execute_trajectory,
)
from common_msgs.msg import MotionCommand

# Same as teach pendant: current pose from TF
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
trans = tf_buffer.lookup_transform("base_link", "gripper_tip_link", rospy.Time(0), rospy.Duration(0.5))
start_xyz = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]

# Current joints from /joint_states (in real code subscribe once or use callback)
start_joints = [...]  # 6-DOF

# Goal from config or upper layer; no IK here
goal_xyz = get_default_virtual_grasp_point()  # or custom [x, y, z]
goal_joints = rospy.get_param("~goal_joints", [0, -2.0, 1.2, -1.57, -1.57, 0])

path_joints, traj = plan_one_shot(
    start_xyz=start_xyz,
    start_joints=start_joints,
    goal_xyz=goal_xyz,
    goal_joints=goal_joints,
    obstacles=None,
    ee_half_extents=None,  # None => ~end_effector_collision_box; or (0,0,0) to disable EE box
)

if path_joints and traj:
    cmd = build_motion_command_execute_trajectory(traj)
    pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
    pub.publish(cmd)
```

This lets you reuse one-shot planning in RoboCup behavior trees or task nodes while staying consistent with the teach pendant and motion_control usage.
