# 🏆 RoboCup UR5e Object Sorting System

[![License: MIT (Non-Commercial)](https://img.shields.io/badge/License-MIT%20(Non--Commercial)-blue.svg)](LICENSE)
[![ROS Version](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](http://wiki.ros.org/noetic)
[![Python Version](https://img.shields.io/badge/Python-3.8-blue.svg)](https://www.python.org/)

**A modular ROS 1 Noetic system for YCB object sorting using UR5e robotic arm**

> **King's College London (KCL) - Robotics Group Project 2026**

---

## 📖 Project Overview

This project implements an autonomous object sorting system for the RoboCup competition. The robot identifies YCB dataset objects, classifies them by color, computes optimal grasp poses, plans collision-free trajectories, and sorts objects into colored bins to maximize scores within a time limit.

**Key Features:**
- 🎯 **Finite State Machine (FSM)** for robust task orchestration
- 👁️ **YOLOv8 Object Detection** with YCB dataset integration
- 🤏 **GraspNet Grasp Estimation** for reliable grasping
- 🛤️ **Path Planning & Collision Avoidance** with MoveIt
- 🤖 **IK/FK Motion Control** for UR5e manipulator
- 🐳 **Dockerized Architecture** for cross-platform development
- ⚡ **CUDA Support** for GPU-accelerated perception (CUDA 11.3 & 12.0)

---

## 👥 Team Members & Responsibilities

| Team Member | Role | Package | Responsibilities |
|-------------|------|---------|------------------|
| **Suhang Xia** | System Architect & FSM | `robocup_brain` | System architecture, FSM implementation, object scoring algorithm, subsystem integration, error recovery |
| **Jiaxin Liang** | Motion Control | `motion_control` | Forward/Inverse kinematics, dynamics, trajectory generation, low-level motion execution |
| **Sarvin & Chang Gao** | Path Planning | `path_planning` | Collision-free path planning, obstacle avoidance, trajectory optimization, MoveIt integration |
| **Fazhan & Ruiyi** | Object Detection | `perception_yolo` | YOLOv8 detection, YCB classification, color recognition, 3D localization, score assignment |
| **Muye Yuan** | Grasp Estimation | `perception_grasp` | GraspNet integration, point cloud processing, grasp quality evaluation, object segmentation |

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                      ROBOCUP_BRAIN (FSM)                             │
│                   Suhang Xia - Architect                             │
│  ┌────────┐  ┌────────┐  ┌───────┐  ┌───────┐  ┌────────┐         │
│  │ SEARCH │─▶│ DETECT │─▶│ SCORE │─▶│ GRASP │─▶│ PLACE  │         │
│  └────────┘  └────────┘  └───────┘  └───────┘  └────────┘         │
│       └────────────────┬────────────────┘                            │
│                   ┌────▼─────┐                                       │
│                   │ RECOVERY │ (Error Handling)                      │
│                   └──────────┘                                       │
└──────────────┬────────────────────────────────┬─────────────────────┘
               │                                │
       ┌───────▼──────┐                ┌───────▼──────┐
       │  PERCEPTION  │                │    MOTION    │
       └──────────────┘                └──────────────┘
               │                                │
   ┌───────────┴────────┐         ┌────────────┴─────────────┐
   │                    │         │                          │
┌──▼────────┐  ┌────────▼───┐  ┌─▼────────┐    ┌──────────▼──┐
│   YOLO    │  │   GRASP    │  │   PATH   │    │   MOTION    │
│ Detection │  │ Estimation │  │ PLANNING │    │  CONTROL    │
│ (CUDA 12) │  │ (CUDA 11.3)│  │          │    │   (IK/FK)   │
└───────────┘  └────────────┘  └──────────┘    └─────────────┘
```

---

## 🚀 Quick Start

### For Team Members

1. **Read the setup guide**: [`docs/SETUP_GUIDE.md`](docs/SETUP_GUIDE.md) - Platform-specific instructions (Ubuntu/WSL2/Mac)
2. **Read the team guide**: [`docs/TEAM_README.md`](docs/TEAM_README.md) - Your specific tasks and interfaces
3. **Download models**: Run `./scripts/download_models.sh` to get YOLO and GraspNet weights
4. **Find your TODOs**: Search for `TODO` comments in your assigned file
5. **Start developing**: Edit code, restart containers, test, commit!

### Quick Commands

```bash
# Clone repository
git clone https://github.com/your-username/robocup_ur5e.git
cd robocup_ur5e

# Download required models (first time only)
./scripts/download_models.sh

# Build Docker images (first time only, 30-60 min)
./scripts/rebuild_all.sh

# Start all services
./scripts/start.sh

# Check system status
./scripts/status.sh

# View logs
docker-compose logs -f

# Enter container with ROS environment ready
docker exec -it robocup_brain bash -c "source /workspace/scripts/setup_container_env.sh && bash"

# Test UR5e arm control
docker exec -it robocup_brain bash -c "source /workspace/scripts/setup_container_env.sh && python3 /workspace/src/robocup_brain/nodes/test_ur5e_control.py"
```

---

## 📦 Project Structure

```
robocup_ur5e/
├── src/                      # ROS packages
│   ├── common_msgs/          # Shared message definitions
│   ├── robocup_brain/        # FSM orchestration (Suhang)
│   ├── motion_control/       # IK/FK/Dynamics (Jiaxin)
│   ├── path_planning/        # Path planning (Sarvin & Chang)
│   ├── perception_yolo/      # Object detection (Fazhan & Ruiyi)
│   └── perception_grasp/     # Grasp estimation (Muye)
├── docker/                   # Docker configurations
├── docs/                     # 📚 All documentation
│   ├── SETUP_GUIDE.md        # ⭐ Platform setup
│   ├── TEAM_README.md        # ⭐ Team tasks
│   └── MODELS_AND_DATASETS.md # ⭐ Model downloads
├── scripts/                  # 🔧 System scripts
│   ├── start.sh              # Start system
│   ├── rebuild_all.sh        # Build images
│   └── download_models.sh    # Download weights
├── weights/                  # 🤖 Model weights (Git LFS / Hugging Face)
│   ├── yolo/                 # YOLO detection models
│   └── graspnet/             # GraspNet checkpoints
└── data/                     # 📊 Datasets (Hugging Face)
    ├── datasets/             # Training data
    └── ycb_objects/          # YCB object models
```

---

## 🔄 ROS Topics

**Perception:**
- `/perception/detected_objects` - Detected YCB objects with scores
- `/perception/grasp_candidates` - Computed grasp poses

**Decision:**
- `/brain/task_decision` - Current FSM state and target
- `/brain/motion_request` - Motion commands

**Motion:**
- `/planning/trajectory` - Planned collision-free trajectories
- `/motion/command` - Low-level motion commands
- `/motion/result` - Execution results

---

## 🛠️ Technology Stack

- **ROS**: ROS 1 Noetic
- **Perception**: YOLOv8 (CUDA 12.0), GraspNet-1Billion (CUDA 11.3)
- **Planning**: MoveIt, OMPL (RRT/RRT*)
- **Control**: UR5e IK/FK, Trajectory Generation
- **Libraries**: PyTorch, OpenCV, Open3D, py_trees_ros
- **Deployment**: Docker, Docker Compose, NVIDIA Container Runtime

---

## 📚 Documentation

| Document | Purpose |
|----------|---------|
| **[docs/SETUP_GUIDE.md](docs/SETUP_GUIDE.md)** ⭐ | Platform-specific setup (Ubuntu/WSL2/Mac) |
| **[docs/TEAM_README.md](docs/TEAM_README.md)** ⭐ | Team member tasks and interfaces |
| **[docs/MODELS_AND_DATASETS.md](docs/MODELS_AND_DATASETS.md)** ⭐ | Model weights and dataset downloads |
| **[docs/CONTRIBUTING.md](docs/CONTRIBUTING.md)** | Development guidelines |
| **[docs/DEPENDENCIES.md](docs/DEPENDENCIES.md)** | Version compatibility |
| **[scripts/README.md](scripts/README.md)** | Script documentation |

---

## 🎓 Academic Use

### Citation

If you use this system in your research or reference it in publications, please cite this repository:

```bibtex
@misc{robocup_ur5e_kcl_2026,
  author = {Suhang Xia and Jiaxin Liang and Sarvin and Chang Gao and Fazhan Liu and Ruiyi Hu and Muye Yuan},
  title = {RoboCup UR5e Object Sorting System},
  year = {2026},
  publisher = {King's College London},
  howpublished = {\url{https://github.com/your-username/robocup_ur5e}},
  note = {RoboCup Competition - YCB Object Sorting}
}
```

### Acknowledgments

This project was developed as part of the Robotics course at **King's College London (KCL)** in 2026. We thank:
- **Suhang Xia** - System architecture and FSM design
- **Jiaxin Liang** - Motion control and kinematics
- **Sarvin & Chang Gao** - Path planning and collision avoidance
- **Fazhan & Ruiyi** - Computer vision and object detection
- **Muye Yuan** - Grasp pose estimation
- All KCL faculty and staff who supported this project

---

## 📜 License

**MIT License (Non-Commercial Use Only)**

Copyright (c) 2026 King's College London - Robotics Team

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction for **non-commercial purposes only**, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, subject to the following conditions:

**The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.**

**THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.**

### ⚠️ Non-Commercial Use Only

This software is licensed for **academic and research purposes only**. **Commercial use is strictly prohibited** without explicit written permission from the authors.


---

## 🤝 Contributing

This is a group project for KCL. Team members should read [`CONTRIBUTING.md`](CONTRIBUTING.md) and follow the development workflow outlined in [`TEAM_README.md`](TEAM_README.md).

---

## 🐛 Issue Reporting

Found a bug? Please open an issue with:
- Description of the problem
- Steps to reproduce
- Expected vs actual behavior
- System information (OS, GPU, Docker version)

---

## 📞 Contact

- **Project Lead**: Suhang Xia - suhang.xia@kcl.ac.uk
- **GitHub Issues**: [Report bugs or request features](https://github.com/your-username/robocup_ur5e/issues)

---

## 🏆 Competition Information

**Event**: RoboCup 2026 - Object Sorting Challenge  
**Task**: YCB Object Sorting by Color  
**Robot**: Universal Robots UR5e  
**Institution**: King's College London (KCL)

---

**Built with ❤️ by the KCL Robotics Team**

*Last Updated: January 26, 2026*
