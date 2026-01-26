# 🤖 Model Weights & Datasets - Download Guide

This document explains how to download and manage model weights and datasets for the RoboCup UR5e system.

---

## 📦 File Size Strategy

| Type | Size | Storage | Git Commit |
|------|------|---------|------------|
| **Config files** | <1MB | Git | ✅ Yes |
| **Small weights** | <10MB | Git | ✅ Yes (YOLOv8-nano) |
| **Large weights** | >10MB | Hugging Face | ❌ No |
| **Datasets** | >100MB | Hugging Face / External | ❌ No |

---

## 🎯 YOLO Weights (Object Detection)

### Recommended Models

| Model | Size | Speed | Accuracy | Use Case |
|-------|------|-------|----------|----------|
| **YOLOv8n** | 6MB | ⚡⚡⚡ Fast | 🎯 Good | Development/Testing |
| **YOLOv8s** | 22MB | ⚡⚡ Medium | 🎯🎯 Better | Production (CPU) |
| **YOLOv8m** | 52MB | ⚡ Slow | 🎯🎯🎯 Best | Production (GPU) |

### Download Instructions

#### Option 1: Auto-download (Recommended)

The system will automatically download YOLOv8n on first run:

```bash
# Start system - will auto-download yolov8n.pt
./scripts/start.sh
```

#### Option 2: Manual download

```bash
# Download YOLOv8 nano (6MB) - small enough to commit to Git
cd weights/yolo/
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Download YOLOv8 small (22MB) - host on Hugging Face
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt

# Download YOLOv8 medium (52MB) - host on Hugging Face
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8m.pt
```

#### Option 3: Download from Hugging Face (Team Shared)

```bash
# Install huggingface-cli
pip install huggingface-hub

# Download from team repository
huggingface-cli download SuhangXia/robocup-ur5e-models yolov8s.pt --local-dir weights/yolo/
```

### YCB Fine-tuned Weights

**Fine-tuned on YCB dataset for competition objects:**

```bash
# TODO: Upload after training
huggingface-cli download SuhangXia/robocup-ur5e-models yolov8s_ycb.pt --local-dir weights/yolo/
```

**Training script**: `src/perception_yolo/train/train_ycb.py` (to be implemented by Fazhan & Ruiyi)

---

## 🤏 GraspNet Weights (Grasp Estimation)

### Model Information

- **Model**: GraspNet-1Billion
- **Size**: ~600MB (checkpoint)
- **Storage**: Hugging Face only
- **Paper**: https://graspnet.net/

### Download Instructions

#### Option 1: Auto-download (Recommended)

```bash
# System will prompt on first run
./scripts/start.sh
# Follow the download instructions in logs
```

#### Option 2: Manual download from Official Source

```bash
cd weights/graspnet/

# Download checkpoint (~600MB)
wget https://graspnet.net/models/checkpoint-rs.tar
tar -xvf checkpoint-rs.tar
rm checkpoint-rs.tar  # Remove tar after extraction
```

#### Option 3: Download from Hugging Face (Team Mirror)

```bash
# Faster for team members
huggingface-cli download SuhangXia/robocup-ur5e-models \
  graspnet_checkpoint.tar \
  --local-dir weights/graspnet/

cd weights/graspnet/
tar -xvf graspnet_checkpoint.tar
```

---

## 📊 YCB Object Dataset

### Dataset Information

- **Name**: YCB Object and Model Set
- **Size**: ~2.5GB (meshes + textures)
- **Objects**: 77 common household objects
- **Use**: Object models for Gazebo simulation and grasp planning

### Download Instructions

#### Option 1: Essential Objects Only (~100MB)

```bash
cd data/ycb_objects/

# Download essential competition objects (curated list)
# TODO: Create and upload curated list
huggingface-cli download SuhangXia/robocup-ur5e-datasets \
  ycb_essential.tar.gz \
  --local-dir data/ycb_objects/

tar -xzvf ycb_essential.tar.gz
```

#### Option 2: Full Dataset (~2.5GB)

```bash
# Download full YCB dataset
git lfs install
git clone https://huggingface.co/datasets/SuhangXia/ycb-objects data/ycb_objects/full/
```

### Object List

See `data/ycb_objects/competition_objects.txt` for the list of objects used in competition.

---

## 🚀 Quick Setup Script

Create a setup script to download all required models:

```bash
./scripts/download_models.sh
```

This will:
1. ✅ Download YOLOv8n (6MB) - committed to Git
2. ⬇️ Download YOLOv8s (22MB) - from Hugging Face
3. ⬇️ Download GraspNet checkpoint (600MB) - from Hugging Face
4. ⬇️ Download essential YCB objects (100MB) - from Hugging Face

**Total download size**: ~722MB (one-time)

---

## 🌐 Setting Up Hugging Face

### Step 1: Create Account

1. Go to https://huggingface.co/join
2. Create account (free)
3. Join team organization: `SuhangXia/robocup-ur5e`

### Step 2: Install CLI

```bash
pip install huggingface-hub
```

### Step 3: Login

```bash
huggingface-cli login
```

Enter your token from: https://huggingface.co/settings/tokens

### Step 4: Download Models

```bash
# List available models
huggingface-cli repo ls SuhangXia/robocup-ur5e-models

# Download specific model
huggingface-cli download SuhangXia/robocup-ur5e-models yolov8s.pt --local-dir weights/yolo/
```

---

## 📤 Uploading Models (Team Leaders Only)

### Upload to Hugging Face

```bash
# Install upload tool
pip install huggingface-hub

# Login (requires write access)
huggingface-cli login

# Upload YOLO weights
huggingface-cli upload SuhangXia/robocup-ur5e-models weights/yolo/yolov8s_ycb.pt yolov8s_ycb.pt

# Upload GraspNet checkpoint
huggingface-cli upload SuhangXia/robocup-ur5e-models weights/graspnet/checkpoint.pth graspnet_checkpoint.pth
```

---

## 📁 Directory Structure

```
robocup_ur5e_ws/
├── weights/                          # Model weights
│   ├── yolo/
│   │   ├── yolov8n.pt               # ✅ Git (6MB)
│   │   ├── yolov8s.pt               # ❌ Hugging Face (22MB)
│   │   ├── yolov8s_ycb.pt           # ❌ Hugging Face (fine-tuned)
│   │   └── config.yaml              # ✅ Git
│   └── graspnet/
│       ├── checkpoint.pth           # ❌ Hugging Face (600MB)
│       ├── config.json              # ✅ Git
│       └── README.md                # ✅ Git
├── data/                             # Datasets
│   ├── datasets/                    # ❌ Hugging Face (training data)
│   └── ycb_objects/
│       ├── competition_objects.txt  # ✅ Git
│       ├── essential/*.obj          # ❌ Hugging Face (100MB)
│       └── full/*.obj               # ❌ Hugging Face (2.5GB)
```

---

## 🔍 Checking Downloaded Models

```bash
# Check YOLO
ls -lh weights/yolo/

# Check GraspNet
ls -lh weights/graspnet/

# Check YCB objects
ls -lh data/ycb_objects/

# Full status
./scripts/status.sh
```

---

## ⚠️ Important Notes

### For Git Users:
- ✅ **Commit**: Config files, small weights (<10MB)
- ❌ **Don't commit**: Large weights, datasets, checkpoints
- ℹ️ `.gitignore` automatically handles this

### For Hugging Face:
- 🔐 **Private repo** for team-only models
- 🌐 **Public repo** for final competition models
- 📦 Use Git LFS for files >10MB

### For Training:
- 💾 Save checkpoints locally during training
- 📤 Upload best checkpoint to Hugging Face after training
- 🗑️ Clean up intermediate checkpoints

---

## 🆘 Troubleshooting

### Problem: "Model not found"

**Solution**:
```bash
# Download missing model
./scripts/download_models.sh

# Or manually download
cd weights/yolo && wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

### Problem: "Hugging Face authentication failed"

**Solution**:
```bash
# Login again with token
huggingface-cli login

# Get token from: https://huggingface.co/settings/tokens
```

### Problem: "Out of disk space"

**Solution**:
```bash
# Remove unnecessary models
rm weights/yolo/yolov8m.pt  # Keep only smaller models

# Remove training checkpoints
rm -rf checkpoints/

# Use smaller YCB object set (essential only)
```

---

## 📞 Contact

- **Model Issues**: Fazhan & Ruiyi (YOLO), Muye Yuan (GraspNet)
- **Dataset Issues**: Suhang Xia
- **Hugging Face Access**: Suhang Xia (suhang@robocup.org)

---

**Last Updated**: January 26, 2026
