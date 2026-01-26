# 📚 Documentation Directory

All project documentation (except main README) is located here.

## 📖 Available Documents

### `SETUP_GUIDE.md` ⭐
**Platform-specific installation instructions**

- Ubuntu 22.04 setup
- Windows WSL2 setup
- macOS setup (CPU-only)
- Docker installation
- GPU configuration
- VirtualBox VM connection

**Read this first if you're setting up your development environment!**

### `TEAM_README.md` ⭐
**Team member tasks and responsibilities**

- System architecture diagrams
- ROS communication graph
- Individual team member assignments
- File locations for each member
- TODO lists and expected code lines
- Interface definitions

**Read this to understand your specific tasks!**

### `MODELS_AND_DATASETS.md` ⭐
**Model weights and dataset management**

- Download instructions for YOLO, GraspNet
- Hugging Face integration
- Git LFS usage
- Storage strategy (Git vs Hugging Face)
- YCB dataset information

**Read this before downloading models!**

### `CONTRIBUTING.md`
**Development guidelines and workflow**

- Git workflow
- Branch naming conventions
- Commit message format
- Code style guidelines
- Pull request process
- Testing requirements

### `DEPENDENCIES.md`
**Software dependencies and version compatibility**

- Python package versions
- ROS package requirements
- CUDA compatibility
- Known version conflicts
- Resolution strategies

### `PUSH_NOW.md`
**GitHub push instructions (for project lead)**

- Final checklist before push
- Push command reference
- Troubleshooting authentication
- Team sharing template

## 📂 Documentation Structure

```
docs/
├── SETUP_GUIDE.md              # ⭐ Start here (platform setup)
├── TEAM_README.md              # ⭐ Your tasks and assignments
├── MODELS_AND_DATASETS.md      # ⭐ Model download guide
├── CONTRIBUTING.md             # Development workflow
├── DEPENDENCIES.md             # Version compatibility
└── PUSH_NOW.md                 # Git push reference
```

## 🎯 Quick Navigation

**I'm a new team member, where do I start?**
1. Read `SETUP_GUIDE.md` for platform setup
2. Read `TEAM_README.md` to find your tasks
3. Read `MODELS_AND_DATASETS.md` to download models
4. Read `CONTRIBUTING.md` for development workflow

**I want to train models, what do I need?**
- Read `MODELS_AND_DATASETS.md` for dataset info
- Check `DEPENDENCIES.md` for training dependencies

**I'm having build/dependency issues?**
- Check `DEPENDENCIES.md` for known conflicts
- Check `SETUP_GUIDE.md` troubleshooting section

**I want to contribute code?**
- Read `CONTRIBUTING.md` for guidelines
- Read `TEAM_README.md` for your interface definitions

## 📝 Adding New Documentation

1. Create `.md` file in `docs/` directory
2. Use clear, descriptive filename
3. Add entry to this README
4. Link from relevant documents
5. Keep language consistent (English only)

## 🔗 External Links

- **Main README**: `../README.md` (project homepage)
- **Scripts**: `../scripts/README.md` (script documentation)
- **Source Code**: `../src/` (ROS packages)

---

**All documentation should be in English. No Chinese characters.**
