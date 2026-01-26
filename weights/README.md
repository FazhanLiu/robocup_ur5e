# Weights Directory

This directory stores model weights and checkpoints for the RoboCup UR5e system.

## Directory Structure

```
weights/
├── yolo/              # YOLO object detection weights
│   ├── yolov8n.pt    # Nano model (6MB) - committed to Git
│   ├── yolov8s.pt    # Small model (22MB) - download required
│   └── config.yaml   # YOLO configuration
└── graspnet/          # GraspNet grasp estimation checkpoints
    ├── checkpoint/   # Extracted checkpoint files
    └── config.json   # GraspNet configuration
```

## Download Models

### Quick Start
```bash
# Auto-download all required models
./scripts/download_models.sh
```

### Manual Download
See `docs/MODELS_AND_DATASETS.md` for detailed instructions.

## Storage Strategy

| File Type | Size | Storage | Committed to Git |
|-----------|------|---------|------------------|
| Config files | <1MB | Git | ✅ Yes |
| Small weights | <10MB | Git | ✅ Yes |
| Large weights | >10MB | Hugging Face | ❌ No |

## Git Ignore Rules

Large weight files (`.pt`, `.pth`, `.ckpt`, `.tar`) are automatically ignored by `.gitignore`.

Only configuration files and small models are committed to the repository.

---

**Note**: First-time users should run `./scripts/download_models.sh` to download required models.
