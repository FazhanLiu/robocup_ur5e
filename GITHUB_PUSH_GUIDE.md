# 🚀 GitHub Push Instructions

## Quick Guide

### Option 1: Automated Script (Recommended)

```bash
cd /home/suhang/robocup_ur5e_ws
./push_to_github.sh
```

The script will:
1. Initialize Git repository
2. Add all files
3. Create initial commit
4. Ask for your GitHub repository URL
5. Add remote and push

---

### Option 2: Manual Steps

#### 1. Create GitHub Repository

Go to https://github.com/new and create a repository:
- **Name**: `robocup_ur5e` (or your preferred name)
- **Description**: "RoboCup UR5e Object Sorting System - KCL Group Project"
- **Visibility**: **Public** ✅
- **DO NOT** initialize with README, .gitignore, or license (we already have them)

#### 2. Initialize Git and Push

```bash
cd /home/suhang/robocup_ur5e_ws

# Initialize
git init

# Add all files
git add .

# Create initial commit
git commit -m "feat: initial RoboCup UR5e system

- Complete ROS Noetic monorepo
- Docker configurations for all services
- Interface definitions and skeleton code
- Comprehensive documentation
- Multi-platform support

Team: KCL Robotics Group 2026"

# Add remote (replace with your URL)
git remote add origin https://github.com/YOUR_USERNAME/robocup_ur5e.git

# Push to GitHub
git branch -M main
git push -u origin main
```

---

## 📋 What Gets Pushed

### ✅ Included:
- All source code (`src/`)
- Docker configurations (`docker/`, `docker-compose.yml`)
- Documentation (`.md` files)
- Launch files, configs
- Shell scripts
- `.gitignore`, `LICENSE`

### ❌ Excluded (via `.gitignore`):
- ROS build artifacts (`build/`, `devel/`, `logs/`)
- Model checkpoints (`*.pt`, `*.pth`, `*.tar`)
- Docker images (push separately if needed)
- Python cache (`__pycache__/`)
- IDE settings

**Total size pushed**: ~100 MB (code only, no models or Docker images)

---

## 🐳 Optional: Push Docker Images to Docker Hub

If you want team members to **pull pre-built images** instead of building locally:

### 1. Create Docker Hub Account

Go to https://hub.docker.com/ and create account

### 2. Login

```bash
docker login
```

### 3. Tag and Push Images

```bash
# Brain image
docker tag robocup_ur5e/brain YOUR_DOCKERHUB_USERNAME/robocup_brain:latest
docker push YOUR_DOCKERHUB_USERNAME/robocup_brain:latest

# YOLO image
docker tag robocup_ur5e/perception_yolo YOUR_DOCKERHUB_USERNAME/robocup_yolo:latest
docker push YOUR_DOCKERHUB_USERNAME/robocup_yolo:latest

# Grasp image
docker tag robocup_ur5e/perception_grasp YOUR_DOCKERHUB_USERNAME/robocup_grasp:latest
docker push YOUR_DOCKERHUB_USERNAME/robocup_grasp:latest
```

⚠️ **Note**: This will upload ~40GB to Docker Hub (may take hours depending on your internet speed)

### 4. Update docker-compose.yml

Tell team members to edit `docker-compose.yml`:

```yaml
services:
  robocup_brain:
    image: YOUR_DOCKERHUB_USERNAME/robocup_brain:latest
    # build:
    #   context: .
    #   dockerfile: docker/Dockerfile.brain
```

Then team members can simply:
```bash
docker-compose pull    # Download pre-built images
docker-compose up -d   # Start services
```

---

## 👥 Share with Team

After pushing to GitHub, send this message to your team:

```
Hi Team,

The RoboCup UR5e system is now on GitHub! 🎉

📦 Repository: https://github.com/YOUR_USERNAME/robocup_ur5e

📖 Getting Started:
1. Read SETUP_GUIDE.md for platform-specific setup
2. Read TEAM_README.md for your specific tasks
3. Clone repo: git clone <URL>
4. Build images: ./rebuild_all.sh (30-60 min first time)
5. Start system: ./start.sh
6. Find your TODOs: grep -n "TODO" src/your_package/nodes/*.py

📋 Quick Links:
- Setup Guide: https://github.com/YOUR_USERNAME/robocup_ur5e/blob/main/SETUP_GUIDE.md
- Team Tasks: https://github.com/YOUR_USERNAME/robocup_ur5e/blob/main/TEAM_README.md
- Architecture: https://github.com/YOUR_USERNAME/robocup_ur5e/blob/main/ARCHITECTURE_SUMMARY.md

All interface code is ready - just fill in your TODO markers!

- Suhang
```

---

## ✅ Verification Checklist

After pushing, verify on GitHub:

- [ ] All source files visible
- [ ] README.md displays correctly on homepage
- [ ] LICENSE file present
- [ ] TEAM_README.md accessible
- [ ] SETUP_GUIDE.md accessible
- [ ] No large model files pushed (check repository size <150MB)
- [ ] Docker images NOT in repository (they're too large)

---

## 🔄 Updating Repository

When you make changes:

```bash
git add <changed_files>
git commit -m "feat(package): description of changes"
git push origin main
```

Team members update:
```bash
git pull origin main
docker-compose restart  # Apply code changes
```

---

## 🆘 Common Issues

### Issue: "Repository not found"

**Solution**: Make sure you created the repository on GitHub first at https://github.com/new

### Issue: "Authentication failed"

**Solution**: Configure Git credentials:
```bash
# Option 1: Personal Access Token (recommended)
# Generate at: https://github.com/settings/tokens
git config --global credential.helper store
git push  # Enter username and token

# Option 2: SSH key
# Follow: https://docs.github.com/en/authentication/connecting-to-github-with-ssh
```

### Issue: "Push rejected - non-fast-forward"

**Solution**: Pull first, then push:
```bash
git pull origin main --rebase
git push origin main
```

---

**Ready to push? Run `./push_to_github.sh` now!** 🚀
