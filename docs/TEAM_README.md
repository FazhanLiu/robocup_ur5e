# 🏆 RoboCup UR5e Object Sorting System - Team Development Guide

**Competition Task**: YCB Object Sorting by Color into Two Bins (Time-limited, Score-based)

---

## 📋 Table of Contents

1. [System Architecture](#system-architecture)
2. [ROS Communication Graph](#ros-communication-graph)
3. [Team Member Responsibilities](#team-member-responsibilities)
4. [Package Overview](#package-overview)
5. [Development Workflow](#development-workflow)
6. [Message Definitions](#message-definitions)
7. [Testing Strategy](#testing-strategy)

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         ROBOCUP_BRAIN (FSM)                              │
│                      Suhang Xia - System Architect                       │
│                                                                           │
│  ┌──────────┐   ┌──────────┐   ┌─────────┐   ┌──────────┐   ┌────────┐│
│  │  SEARCH  │──▶│  DETECT  │──▶│  SCORE  │──▶│  GRASP   │──▶│ PLACE  ││
│  └──────────┘   └──────────┘   └─────────┘   └──────────┘   └────────┘│
│       │              │               │              │             │     │
│       └──────────────┴───────────────┴──────────────┴─────────────┘     │
│                                  │                                       │
│                            ┌─────▼──────┐                               │
│                            │  RECOVERY  │◀─── Error Handling            │
│                            └────────────┘                               │
└──────────────┬──────────────────────────────────────────┬───────────────┘
               │                                          │
       ┌───────▼────────┐                        ┌───────▼────────┐
       │   PERCEPTION   │                        │    MOTION      │
       └────────────────┘                        └────────────────┘
               │                                          │
   ┌───────────┴───────────┐              ┌──────────────┴──────────────┐
   │                       │              │                             │
┌──▼───────┐      ┌────────▼────┐  ┌─────▼──────┐         ┌───────────▼──┐
│ YOLO     │      │ GRASP       │  │ PATH       │         │  MOTION      │
│ Detector │      │ Estimator   │  │ PLANNING   │         │  CONTROL     │
│          │      │             │  │            │         │   (IK/FK)    │
│ Fazhan & │      │ Muye Yuan   │  │ Sarvin &   │         │ Jiaxin Liang │
│ Ruiyi    │      │             │  │ Chang Gao  │         │              │
└──────────┘      └─────────────┘  └────────────┘         └──────────────┘
    │                   │                 │                       │
    │ RGB Image         │ Point Cloud     │ Planned Path         │ Joint Cmds
    │                   │                 │                       │
┌───▼───────────────────▼─────────────────▼───────────────────────▼────────┐
│                         UR5e ROBOT + SENSORS                              │
│                   (Gazebo Simulation in VirtualBox VM)                    │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## 🔄 ROS Communication Graph

```
┌──────────────────────────────────────────────────────────────────────────┐
│                            ROS TOPICS & SERVICES                          │
└──────────────────────────────────────────────────────────────────────────┘

                                   ┌──────────────┐
                                   │  /camera/*   │
                                   └──────┬───────┘
                                          │
                         ┌────────────────┴────────────────┐
                         │                                 │
                    Image (RGB)                    PointCloud2 (Depth)
                         │                                 │
                         │                                 │
              ┌──────────▼──────────┐           ┌─────────▼──────────┐
              │  perception_yolo    │           │ perception_grasp   │
              │  (YOLO Detector)    │           │ (Grasp Estimator)  │
              └──────────┬──────────┘           └─────────┬──────────┘
                         │                                 │
                         │                                 │
             /perception/detected_objects        /perception/grasp_candidates
                    (ObjectScore[])                   (GraspCandidate[])
                         │                                 │
                         └─────────────┬───────────────────┘
                                       │
                              ┌────────▼─────────┐
                              │  robocup_brain   │
                              │  (FSM + Scorer)  │
                              └────────┬─────────┘
                                       │
                      ┌────────────────┼────────────────┐
                      │                │                │
             /brain/task_decision      │       /brain/motion_request
              (TaskDecision)           │         (MotionCommand)
                      │                │                │
                      │     /planning/path_request      │
                      │      (PathPlanRequest)          │
                      │                │                │
              ┌───────▼────────┐  ┌────▼───────┐  ┌────▼──────────┐
              │ path_planning  │  │   path_    │  │    motion_    │
              │ (MoveIt/Custom)│  │  planning  │  │    control    │
              └───────┬────────┘  └────┬───────┘  └────┬──────────┘
                      │                │                │
                      │                │                │
              /planning/trajectory     │       /motion/command
               (JointTrajectory)       │       (MotionCommand)
                                       │                │
                                       │                │
                                       └────────┬───────┘
                                                │
                                    ┌───────────▼────────────┐
                                    │  /joint_trajectory_    │
                                    │  controller/command    │
                                    └───────────┬────────────┘
                                                │
                                         ┌──────▼──────┐
                                         │  UR5e Robot │
                                         │  Controller │
                                         └─────────────┘

KEY SERVICES:
  • /motion/compute_ik      : IK request/response (motion_control)
  • /motion/compute_fk      : FK request/response (motion_control)
  • /planning/plan_path     : Path planning service (path_planning)
  • /grasp/segment_pointcloud : Point cloud segmentation (perception_grasp)
```

---

## 👥 Team Member Responsibilities

### 🎯 **1. Suhang Xia  - System Architect & FSM**

**Package**: `robocup_brain`

**File to Develop**: `src/robocup_brain/nodes/brain_fsm_node.py` (Will be created)

**Responsibilities**:
1. **Finite State Machine (FSM)**: Implement robust state machine to orchestrate the entire system
2. **Object Scoring Algorithm**: Implement weighted scoring function:
   ```
   total_score = w1 * base_score + w2 * (1/distance) + w3 * (1/grasp_difficulty)
   ```
3. **Decision Making**: Select next best object to pick based on scores
4. **System Integration**: Connect all subsystems and handle their responses
5. **Error Recovery**: Implement recovery strategies for failures (re-scan, skip object, etc.)
6. **Time Management**: Track competition time and optimize decisions

**Key Functions to Implement** (Empty interfaces ready):
```python
def compute_object_scores(detected_objects, grasp_candidates) -> List[ObjectScore]:
    """
    TODO: Implement scoring algorithm
    - Get base_score from YCB dataset rules
    - Compute distance from robot
    - Get grasp_difficulty from grasp estimator
    - Apply weights and compute total_score
    """
    pass

def select_next_target(scored_objects) -> Optional[ObjectScore]:
    """
    TODO: Implement selection logic
    - Sort by total_score
    - Check if object is reachable
    - Avoid recently failed objects
    """
    pass

def fsm_state_search() -> str:
    """TODO: Implement search behavior"""
    pass

def fsm_state_approach() -> str:
    """TODO: Implement approach behavior"""
    pass

def fsm_state_grasp() -> str:
    """TODO: Implement grasp execution"""
    pass

def fsm_state_transport() -> str:
    """TODO: Implement transport to bin"""
    pass

def fsm_state_release() -> str:
    """TODO: Implement release in bin"""
    pass

def fsm_state_recover() -> str:
    """TODO: Implement error recovery"""
    pass
```

**Lines of Code**: ~400 lines (FSM + Scoring + Integration)

**Container**: `robocup_brain` (already running)

**Topics**:
- Subscribe: `/perception/detected_objects`, `/perception/grasp_candidates`, `/motion/result`, `/planning/result`
- Publish: `/brain/task_decision`, `/brain/motion_request`, `/brain/planning_request`

---

### 🤖 **2. Jiaxin Liang - Motion Control (IK/FK/Dynamics)**

**Package**: `motion_control`

**File to Develop**: `src/motion_control/nodes/motion_control_node.py` (**ALREADY CREATED**)

**Responsibilities**:
1. **Forward Kinematics (FK)**: Implement joint angles → end-effector pose
2. **Inverse Kinematics (IK)**: Implement end-effector pose → joint angles (use UR5e analytical IK)
3. **Dynamics**: Compute joint torques, velocities, accelerations
4. **Trajectory Generation**: Generate smooth joint/Cartesian trajectories
5. **Motion Execution**: Interface with robot controller for low-level execution

**Key Functions to Implement** (Empty interfaces ready - see file):
```python
def compute_forward_kinematics(joint_angles) -> PoseStamped:
    """TODO: Implement FK using UR5e kinematics"""
    pass

def compute_inverse_kinematics(target_pose, seed_joints) -> List[float]:
    """TODO: Implement IK (use ur_kinematics or custom analytical IK)"""
    pass

def compute_jacobian(joint_angles) -> np.ndarray:
    """TODO: Compute 6x6 Jacobian matrix"""
    pass

def generate_joint_trajectory(start, goal, duration) -> JointTrajectory:
    """TODO: Generate smooth trajectory (quintic polynomial or spline)"""
    pass

def _execute_cartesian_motion(target_pose, cmd):
    """TODO: Execute Cartesian space motion"""
    pass

def _execute_joint_motion(target_joints, cmd):
    """TODO: Execute joint space motion"""
    pass
```

**Lines of Code**: ~500 lines (already provided as skeleton)

**Container**: New container `motion_control` (needs Docker config - see below)

**Topics**:
- Subscribe: `/joint_states`, `/motion/command`
- Publish: `/motion/result`, `/joint_trajectory`
- Services: `/motion/compute_ik`, `/motion/compute_fk`

---

### 🛤️ **3. Sarvin & Chang Gao - Path Planning & Collision Avoidance**

**Package**: `path_planning`

**File to Develop**: `src/path_planning/nodes/path_planner_node.py` (Will be created)

**Responsibilities**:
1. **Path Planning**: Use MoveIt or custom planner (RRT, RRT*, PRM) to find collision-free paths
2. **Collision Avoidance**: Ensure robot doesn't collide with table, bins, or other objects
3. **Trajectory Optimization**: Smooth and time-optimize planned paths
4. **Dynamic Replanning**: Replan if environment changes

**Key Functions to Implement** (Empty interfaces will be provided):
```python
def plan_to_pose(start_pose, goal_pose, constraints) -> JointTrajectory:
    """
    TODO: Implement path planning
    - Use MoveIt's planning interface or
    - Implement custom sampling-based planner (RRT/RRT*)
    - Check collisions with scene
    - Return smooth trajectory
    """
    pass

def add_collision_object(name, pose, dimensions):
    """TODO: Add obstacle to planning scene"""
    pass

def remove_collision_object(name):
    """TODO: Remove obstacle from planning scene"""
    pass

def compute_cartesian_path(waypoints, step_size) -> JointTrajectory:
    """TODO: Plan path through Cartesian waypoints"""
    pass
```

**Lines of Code**: ~350 lines

**Container**: Can share `robocup_brain` container or create separate

**Topics**:
- Subscribe: `/planning/path_request`
- Publish: `/planning/trajectory`, `/planning/result`
- Services: `/planning/plan_to_pose`, `/planning/add_obstacle`

---

### 👁️ **4. Fazhan & Ruiyi - YOLO Object Detection & Scoring**

**Package**: `perception_yolo`

**File to Develop**: `src/perception_yolo/nodes/yolo_detector_node.py` (**ALREADY CREATED**)

**Responsibilities**:
1. **YOLOv8 Integration**: Load and run YOLOv8 model for YCB object detection
2. **Object Recognition**: Detect and classify objects from camera images
3. **Color Classification**: Determine object color for bin sorting (red vs blue)
4. **Score Assignment**: Assign base_score based on competition rules
5. **3D Localization**: Estimate 3D position using depth information

**Key Functions to Implement** (Empty interfaces ready - see file):
```python
def _load_model():
    """
    TODO: Load YOLOv8 model
    - Download YCB-trained weights or fine-tune on YCB dataset
    - Initialize model on CUDA/CPU
    """
    pass

def image_callback(msg: Image):
    """
    TODO: Implement detection pipeline
    1. Convert ROS Image to CV2/NumPy
    2. Run YOLO inference
    3. For each detection:
       - Extract bounding box (ROI)
       - Classify color (red/blue) - may need color thresholding
       - Lookup base_score from YCB dataset rules
       - Estimate 3D position (use depth image)
       - Publish ObjectScore message
    """
    pass

def classify_color(roi_image) -> str:
    """
    TODO: Implement color classification
    - Convert to HSV color space
    - Threshold for red/blue
    - Return "red" or "blue"
    """
    pass

def estimate_3d_position(roi, depth_image) -> Point:
    """
    TODO: Compute 3D position in camera frame
    - Use depth at ROI center
    - Convert 2D pixel + depth to 3D point
    - Transform to world frame using TF
    """
    pass
```

**Lines of Code**: ~200 lines (skeleton provided, ~100 lines to add)

**Container**: `perception_yolo` (already running)

**Topics**:
- Subscribe: `/camera/color/image_raw`, `/camera/depth/image_raw`
- Publish: `/perception/detected_objects` (ObjectScore messages)

**Config File**: `src/perception_yolo/config/ycb_scores.yaml` (Will be created with YCB score mapping)

---

### 🤏 **5. Muye Yuan - Grasp Pose Estimation**

**Package**: `perception_grasp`

**File to Develop**: `src/perception_grasp/nodes/grasp_estimator_node.py` (**ALREADY CREATED**)

**Responsibilities**:
1. **Grasp Algorithm Integration**: Integrate GraspNet-1Billion or alternative grasp estimation algorithm
2. **Point Cloud Processing**: Segment and process point clouds for grasp detection
3. **Grasp Quality Scoring**: Compute grasp_difficulty metric for decision making
4. **Object Segmentation**: Segment target object from scene (optional but recommended)

**Key Functions to Implement** (Empty interfaces ready - see file):
```python
def _load_graspnet_model():
    """
    TODO: Load GraspNet-1Billion model
    - Download checkpoint from GraspNet repository
    - Initialize model on CUDA 11.3
    - Or use alternative grasp algorithm (GPD, 6-DOF GraspNet, etc.)
    """
    pass

def pointcloud_callback(msg: PointCloud2):
    """
    TODO: Implement grasp detection pipeline
    1. Convert PointCloud2 to Open3D format
    2. (Optional) Segment target object from scene
       - Use ROI from detected_objects
       - Crop point cloud to object region
    3. Run grasp detection algorithm
       - GraspNet: returns grasp poses with scores
    4. Select top N grasps
    5. Compute grasp_difficulty (1 - grasp_score)
    6. Publish GraspCandidate messages
    """
    pass

def segment_object_pointcloud(full_cloud, roi) -> o3d.geometry.PointCloud:
    """
    TODO: Segment object point cloud
    - Project ROI to 3D space
    - Crop point cloud
    - Apply statistical outlier removal
    """
    pass

def run_grasp_detection(object_cloud) -> List[GraspCandidate]:
    """
    TODO: Run grasp algorithm
    - Preprocess point cloud (voxel downsampling, normal estimation)
    - Run GraspNet inference
    - Post-process results (collision checking, reachability)
    - Return top candidates
    """
    pass
```

**Lines of Code**: ~250 lines (skeleton provided, ~100 lines to add)

**Container**: `perception_grasp` (already running)

**Topics**:
- Subscribe: `/camera/depth/points`, `/perception/detected_objects`
- Publish: `/perception/grasp_candidates`

**Note**: Coordinate with **Fazhan & Ruiyi** to sync object detection ROIs for point cloud segmentation

---

## 📦 Package Overview

### Package Structure

```
src/
├── common_msgs/              # ✅ COMPLETE - Shared message definitions
│   ├── msg/
│   │   ├── DetectedObject.msg
│   │   ├── GraspCandidate.msg
│   │   ├── ObjectScore.msg        # NEW
│   │   ├── TaskDecision.msg       # NEW
│   │   ├── GraspResult.msg        # NEW
│   │   ├── MotionCommand.msg      # NEW
│   │   └── PathPlanRequest.msg    # NEW
│   ├── CMakeLists.txt
│   └── package.xml
│
├── robocup_brain/            # 🔄 TO BE REDESIGNED (Suhang)
│   ├── nodes/
│   │   └── brain_fsm_node.py      # NEW - FSM + Scoring
│   ├── launch/
│   │   └── brain.launch
│   ├── config/
│   │   ├── scoring_weights.yaml   # NEW - Scoring parameters
│   │   └── ycb_base_scores.yaml   # NEW - YCB competition scores
│   └── package.xml
│
├── motion_control/           # ✅ SKELETON PROVIDED (Jiaxin Liang)
│   ├── nodes/
│   │   └── motion_control_node.py # ~500 lines with TODO markers
│   ├── launch/
│   │   └── motion_control.launch
│   ├── config/
│   │   └── ur5e_config.yaml
│   └── package.xml
│
├── path_planning/            # 🔄 TO BE CREATED (Sarvin & Chang Gao)
│   ├── nodes/
│   │   └── path_planner_node.py   # ~350 lines (skeleton will be provided)
│   ├── launch/
│   │   └── path_planning.launch
│   ├── config/
│   │   └── planning_config.yaml
│   └── package.xml
│
├── perception_yolo/          # 🔄 UPDATE FOR YCB (Fazhan & Ruiyi)
│   ├── nodes/
│   │   └── yolo_detector_node.py  # ~200 lines (add color + scoring)
│   ├── config/
│   │   ├── yolo_config.yaml
│   │   └── ycb_scores.yaml        # NEW - YCB score mapping
│   ├── launch/
│   └── package.xml
│
└── perception_grasp/         # 🔄 INTEGRATE GRASPNET (Muye Yuan)
    ├── nodes/
    │   └── grasp_estimator_node.py # ~250 lines (add GraspNet)
    ├── config/
    │   └── grasp_config.yaml
    ├── launch/
    └── package.xml
```

### Lines of Code Summary

| Team Member | Package | File | Total Lines | Lines to Write | Difficulty |
|-------------|---------|------|-------------|----------------|------------|
| **Suhang Xia** | robocup_brain | brain_fsm_node.py | ~400 | ~300 | ⭐⭐⭐⭐ |
| **Jiaxin Liang** | motion_control | motion_control_node.py | ~500 | ~350 | ⭐⭐⭐⭐⭐ |
| **Sarvin & Chang Gao** | path_planning | path_planner_node.py | ~350 | ~250 | ⭐⭐⭐⭐ |
| **Fazhan & Ruiyi** | perception_yolo | yolo_detector_node.py | ~200 | ~100 | ⭐⭐⭐ |
| **Muye Yuan** | perception_grasp | grasp_estimator_node.py | ~250 | ~100 | ⭐⭐⭐ |

---

## 💬 Message Definitions

### New Messages (in `common_msgs`)

#### 1. **ObjectScore.msg**
```
string label                      # Object class name from YCB dataset
int32 object_id                   # Unique ID for tracking
string color                      # "red" or "blue"
sensor_msgs/RegionOfInterest roi  # Bounding box
geometry_msgs/Point position      # 3D position
float32 confidence                # Detection confidence
float32 base_score                # Base score from competition rules
float32 distance                  # Distance to robot
float32 grasp_difficulty          # Grasp difficulty [0-1]
float32 total_score               # Weighted final score
```

#### 2. **TaskDecision.msg**
```
string task_type                  # "search", "approach", "grasp", "transport", "release", "recover"
int32 target_object_id            # Selected object ID
string target_bin                 # "red" or "blue"
geometry_msgs/PoseStamped target_pose
float32 timeout
int32 retry_count
```

#### 3. **MotionCommand.msg**
```
string command_type               # "move_to_pose", "move_to_joint", "execute_trajectory", "stop", "home"
geometry_msgs/PoseStamped target_pose
float64[] joint_positions
trajectory_msgs/JointTrajectory trajectory
float32 max_velocity
float32 max_acceleration
bool collision_check
```

#### 4. **GraspResult.msg**
```
uint8 status                      # SUCCESS=0, NO_GRASP_FOUND=1, UNREACHABLE=2, COLLISION=3, EXECUTION_FAILED=4
string message
geometry_msgs/PoseStamped grasp_pose
float32 grasp_quality
float32 execution_time
```

#### 5. **PathPlanRequest.msg**
```
geometry_msgs/PoseStamped start_pose
geometry_msgs/PoseStamped goal_pose
string planning_group
float32 planning_time
int32 num_attempts
bool avoid_collisions
string[] obstacle_ids
```

---

## 🔧 Development Workflow

### For Each Team Member:

#### 1. **Clone Repository** (After leader pushes)
```bash
git clone https://github.com/your-username/robocup_ur5e.git
cd robocup_ur5e
```

#### 2. **Start Development Environment**
```bash
./start.sh  # Starts all Docker containers
```

#### 3. **Develop Your Package**

**Edit code on host machine:**
```bash
# Example: Jiaxin working on motion_control
cd src/motion_control/nodes
vim motion_control_node.py

# Find all TODO markers:
grep -n "TODO" motion_control_node.py
```

**Test in container:**
```bash
# Restart your container to apply changes
docker-compose restart motion_control  # (once container is added)

# View logs
docker-compose logs -f motion_control

# Enter container for debugging
docker-compose exec motion_control bash
source /workspace/devel/setup.bash
rosrun motion_control motion_control_node.py
```

#### 4. **Test Integration with Other Nodes**

```bash
# Check if your topics are publishing
rostopic list

# Echo your output
rostopic echo /your_topic_name

# Check message structure
rostopic info /your_topic_name
rosmsg show common_msgs/ObjectScore
```

#### 5. **Commit and Push**

```bash
git add src/your_package/
git commit -m "feat(package): implement feature X"
git push origin main
```

#### 6. **Pull Updates from Team**

```bash
git pull origin main
docker-compose restart  # Apply updates
```

---

## 🧪 Testing Strategy

### Unit Testing (Individual Nodes)

Each member should test their node independently:

1. **Create mock publishers** for input topics
2. **Subscribe to output** topics
3. **Verify correctness** of outputs

Example for Jiaxin (motion_control):
```python
# Test IK
rosservice call /motion/compute_ik "target_pose: ..."

# Test FK
rosservice call /motion/compute_fk "joint_angles: [...]"
```

### Integration Testing (System-level)

Once all nodes are implemented:

1. **Start all containers**:
   ```bash
   ./start.sh
   ```

2. **Start Gazebo simulation** (in VirtualBox VM):
   ```bash
   roslaunch arm_gazebo arm_world.launch
   ```

3. **Monitor system behavior**:
   ```bash
   # View FSM state transitions
   rostopic echo /brain/task_decision

   # View object detections
   rostopic echo /perception/detected_objects

   # View grasp candidates
   rostopic echo /perception/grasp_candidates

   # View planned paths
   rostopic echo /planning/trajectory
   ```

4. **Use RViz** for visualization:
   ```bash
   roslaunch arm_moveit_config moveit_rviz.launch
   ```

---

## 🚀 Quick Start Commands

### Start System
```bash
cd /home/suhang/robocup_ur5e_ws
./start.sh
```

### Check System Status
```bash
./status.sh
```

### View Logs
```bash
docker-compose logs -f                # All nodes
docker-compose logs -f brain          # Single node
```

### Stop System
```bash
docker-compose down
```

### Rebuild After Changes
```bash
docker-compose build --no-cache <service_name>
```

---

## 📞 Communication Protocol

### Team Coordination

1. **Interface Contracts**: All ROS topics and messages are defined in `common_msgs` - **DO NOT change without team agreement**

2. **Code Reviews**: Create pull requests for major features, request review from architect (Suhang)

3. **Integration Points**:
   - **Week 1**: Individual node skeletons ready
   - **Week 2**: Unit tests passing
   - **Week 3**: Integration testing
   - **Week 4**: System optimization and competition prep

4. **Daily Standups** (Recommended):
   - What did I complete?
   - What am I working on today?
   - Any blockers?

---

## 📚 Additional Resources

### For Jiaxin (Motion Control)
- UR5e URDF: Check `arm_description` package in VM
- IK/FK Library: `ur_kinematics` or `KDL` (Kinematics and Dynamics Library)
- Trajectory Generation: Quintic polynomial or cubic spline interpolation

### For Sarvin & Chang Gao (Path Planning)
- MoveIt Tutorials: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/
- RRT/RRT* Algorithms: OMPL library (Open Motion Planning Library)
- Collision Checking: FCL (Flexible Collision Library)

### For Fazhan & Ruiyi (YOLO)
- YOLOv8: https://github.com/ultralytics/ultralytics
- YCB Dataset: http://www.ycbbenchmarks.com/
- Color Classification: OpenCV HSV thresholding

### For Muye Yuan (Grasp Estimation)
- GraspNet-1Billion: https://graspnet.net/
- Alternative: GPD (Grasp Pose Detection), 6-DOF GraspNet
- Point Cloud Processing: Open3D library

### For Suhang (FSM & Architecture)
- FSM Design: State pattern, state machines in Python
- ROS Integration: `rospy`, action clients, service proxies
- System Robustness: Timeouts, retries, error handling

---

## ⚠️ Important Notes

1. **All comments in code MUST be in English** - No Chinese characters

2. **DO NOT modify `common_msgs` without team agreement** - These are shared interfaces

3. **Test your node independently first** before integration

4. **Follow ROS naming conventions**:
   - Topics: `/namespace/topic_name`
   - Nodes: `package_name_node`
   - Messages: `CamelCase`

5. **Handle errors gracefully** - Don't crash, publish error status instead

6. **Log informatively**:
   ```python
   rospy.loginfo("[NodeName] Important info")
   rospy.logwarn("[NodeName] Warning")
   rospy.logerr("[NodeName] Error occurred")
   ```

---

## 🎯 Success Criteria

System is ready when:
- ✅ All nodes start without errors
- ✅ All ROS topics are publishing at expected rates
- ✅ Object detection identifies YCB objects with >80% accuracy
- ✅ Grasp estimation provides valid grasp poses
- ✅ Path planning generates collision-free trajectories
- ✅ Motion control executes trajectories smoothly
- ✅ FSM handles normal workflow and recovers from errors
- ✅ System completes object sorting task in simulation

---

**Good luck, team! Let's win this competition! 🏆**

---

*Last Updated: 2026-01-26*  
*System Architect: Suhang Xia*
