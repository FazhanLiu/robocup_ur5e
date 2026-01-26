# 📜 Scripts Directory

All system management scripts are located here.

## 🚀 Core Scripts

### `start.sh`
**Start the entire system**

```bash
./scripts/start.sh
```

- Checks Docker images
- Verifies configuration
- Starts all containers
- Shows system status

### `rebuild_all.sh`
**Rebuild all Docker images**

```bash
./scripts/rebuild_all.sh
```

- Rebuilds brain, YOLO, and GraspNet containers
- Cleans cache if needed
- Reports build status

### `status.sh`
**Check system status**

```bash
./scripts/status.sh
```

Shows:
- Docker images
- Container status
- ROS topics
- GPU availability

### `check_running.sh`
**Check running containers and logs**

```bash
./scripts/check_running.sh
```

Shows:
- Container uptime
- Recent logs
- ROS node status

## 🔧 Utility Scripts

### `setup_container_env.sh`
**Setup ROS environment inside containers** ⭐

```bash
# Inside container
source /workspace/scripts/setup_container_env.sh
```

This script:
- Sources ROS Noetic and workspace
- Configures ROS network (`ROS_MASTER_URI`, `ROS_IP`)
- Tests ROS Master connection
- Shows available topics

**Use this every time you enter a container!**

### `download_models.sh`
**Download required models and datasets**

```bash
./scripts/download_models.sh
```

Downloads:
- YOLO weights (YOLOv8n, YOLOv8s)
- GraspNet checkpoint (~600MB)
- YCB object models (optional)

### `fix_git_and_push.sh`
**Fix Git configuration and push to GitHub**

```bash
./scripts/fix_git_and_push.sh
```

- Configures Git identity
- Creates commit
- Pushes to remote

## 📋 Usage Examples

### First-Time Setup
```bash
# 1. Download models
./scripts/download_models.sh

# 2. Build Docker images
./scripts/rebuild_all.sh

# 3. Start system
./scripts/start.sh
```

### Development Workflow
```bash
# Check status
./scripts/status.sh

# View logs
./scripts/check_running.sh

# Restart after code changes
docker-compose restart <service>

# Rebuild if Dockerfile changed
./scripts/rebuild_all.sh
```

### Debugging
```bash
# View real-time logs
docker-compose logs -f

# Enter container and setup ROS environment
docker exec -it robocup_brain bash -c "source /workspace/scripts/setup_container_env.sh && bash"

# Quick ROS test
docker exec -it robocup_brain bash -c "source /workspace/scripts/setup_container_env.sh && rostopic list"

# Run arm control test
docker exec -it robocup_brain bash -c "source /workspace/scripts/setup_container_env.sh && python3 /workspace/src/robocup_brain/nodes/test_ur5e_control.py"
```

## 🔐 Permissions

All scripts are executable (`chmod +x`). If you add new scripts:

```bash
chmod +x scripts/your_new_script.sh
```

## 📝 Adding New Scripts

1. Create script in `scripts/` directory
2. Add shebang: `#!/bin/bash`
3. Make executable: `chmod +x scripts/your_script.sh`
4. Document here
5. Test before committing

---

**For detailed system documentation, see `docs/` directory.**
