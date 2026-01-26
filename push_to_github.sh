#!/bin/bash
# Initialize Git repository and push to GitHub

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║         Git Repository Initialization & Push                 ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Check if already initialized
if [ -d ".git" ]; then
    echo "⚠️  Git repository already exists!"
    echo ""
    read -p "Do you want to re-initialize? (yes/no): " reinit
    if [ "$reinit" != "yes" ]; then
        echo "Aborted."
        exit 0
    fi
    rm -rf .git
fi

echo "📝 Step 1: Initialize Git repository..."
git init
echo "✅ Done"
echo ""

echo "📝 Step 2: Add all files..."
git add .
echo "✅ Done"
echo ""

echo "📝 Step 3: Create initial commit..."
git commit -m "feat: initial RoboCup UR5e system with complete architecture

- Complete ROS 1 Noetic monorepo structure
- All Docker configurations (Brain, YOLO, Grasp)
- Interface definitions in common_msgs
- Skeleton code with TODO markers for all team members
- Comprehensive documentation (TEAM_README, SETUP_GUIDE, etc.)
- Multi-platform support (Ubuntu/WSL2/Mac)
- GPU acceleration (CUDA 11.3 & 12.0) with CPU fallback

Team: Suhang Xia, Jiaxin Liang, Sarvin, Chang Gao, Fazhan, Ruiyi, Muye Yuan
Institution: King's College London (KCL)
Competition: RoboCup 2026 - YCB Object Sorting"

echo "✅ Done"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📌 Next: Add GitHub remote and push"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Please provide your GitHub repository URL:"
echo "  Example: https://github.com/your-username/robocup_ur5e.git"
echo ""
read -p "GitHub repository URL: " repo_url

if [ -z "$repo_url" ]; then
    echo "❌ No URL provided. Exiting."
    echo ""
    echo "To push later, run:"
    echo "  git remote add origin <your-repo-url>"
    echo "  git branch -M main"
    echo "  git push -u origin main"
    exit 0
fi

echo ""
echo "📝 Step 4: Add remote repository..."
git remote add origin "$repo_url"
echo "✅ Done"
echo ""

echo "📝 Step 5: Rename branch to main..."
git branch -M main
echo "✅ Done"
echo ""

echo "📝 Step 6: Push to GitHub..."
echo ""
echo "⚠️  This will upload ~100MB of code (Docker images are NOT uploaded)"
echo ""
read -p "Proceed with push? (yes/no): " proceed

if [ "$proceed" = "yes" ]; then
    git push -u origin main
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "✅ Successfully pushed to GitHub!"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "🎉 Your repository is now public at:"
        echo "   $repo_url"
        echo ""
        echo "📋 Next steps:"
        echo "  1. Visit your GitHub repo and verify files"
        echo "  2. Share the repository URL with your team"
        echo "  3. Team members should read SETUP_GUIDE.md"
        echo "  4. Everyone should read TEAM_README.md for their tasks"
        echo ""
        echo "Optional: Push Docker images to Docker Hub"
        echo "  Run: ./push_to_dockerhub.sh"
        echo ""
    else
        echo ""
        echo "❌ Push failed! Common causes:"
        echo "  1. Authentication failed (configure GitHub credentials)"
        echo "  2. Repository doesn't exist (create it on GitHub first)"
        echo "  3. No push access (check repository permissions)"
        echo ""
        echo "To push later:"
        echo "  git push -u origin main"
    fi
else
    echo ""
    echo "Aborted. To push later, run:"
    echo "  git push -u origin main"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📚 Repository Contents:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  README.md              - Project homepage"
echo "  SETUP_GUIDE.md         - Platform-specific setup"
echo "  TEAM_README.md         - Team tasks and interfaces"
echo "  LICENSE                - MIT (Non-Commercial)"
echo "  src/                   - All ROS packages"
echo "  docker/                - Docker configurations"
echo "  docker-compose.yml     - Container orchestration"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
