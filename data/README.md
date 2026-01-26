# Data Directory

This directory stores datasets and object models for the RoboCup UR5e system.

## Directory Structure

```
data/
├── datasets/          # Training and validation datasets
│   ├── yolo/         # YOLO training data
│   └── grasp/        # Grasp training data
└── ycb_objects/       # YCB object models
    ├── essential/    # Competition objects only (~100MB)
    ├── full/         # All 77 YCB objects (~2.5GB)
    └── list.txt      # Object list for competition
```

## Download Datasets

### Quick Start
```bash
# Download essential datasets
./scripts/download_models.sh
```

### Manual Download
See `docs/MODELS_AND_DATASETS.md` for detailed instructions.

## YCB Object Dataset

The YCB Object and Model Set contains 77 everyday objects used for benchmarking grasping and manipulation.

**Competition Objects**: A curated list of objects used in the RoboCup competition (see `list.txt`).

## Storage Strategy

| Data Type | Size | Storage | Committed to Git |
|-----------|------|---------|------------------|
| Object lists | <1MB | Git | ✅ Yes |
| Essential objects | ~100MB | Hugging Face | ❌ No |
| Full dataset | ~2.5GB | Hugging Face | ❌ No |
| Training data | Variable | Hugging Face | ❌ No |

## Git Ignore Rules

All large data files (`.ply`, `.obj`, `.stl`, `.npy`, `.npz`) are automatically ignored.

Only lightweight metadata and lists are committed to the repository.

---

**Note**: Datasets are downloaded on-demand. See documentation for details.
