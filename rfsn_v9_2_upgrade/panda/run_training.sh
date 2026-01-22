#!/bin/bash
#
# Quick Start Script for Panda MuJoCo Bandit Training
# RFSN v9.2
#
# Prerequisites:
#   - Python 3.10+ (for prebuilt mujoco wheels)
#   - pip install mujoco numpy vowpalwabbit
#
# Usage:
#   ./panda/run_training.sh [episodes] [--render]
#
# Examples:
#   ./panda/run_training.sh              # 100 episodes, no render
#   ./panda/run_training.sh 50 --render  # 50 episodes with visualization

set -e

cd "$(dirname "$0")/.."

EPISODES=${1:-100}
RENDER_FLAG=${2:-}

echo "=============================================="
echo "RFSN v9.2 Panda Bandit Training"
echo "=============================================="
echo "Episodes: $EPISODES"
echo "Render: ${RENDER_FLAG:-disabled}"
echo ""

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2 | cut -d'.' -f1,2)
echo "Python version: $PYTHON_VERSION"

# Check dependencies
if ! python3 -c "import mujoco" 2>/dev/null; then
    echo ""
    echo "ERROR: mujoco not installed"
    echo ""
    echo "To install (requires Python 3.10+):"
    echo "  pyenv install 3.10.12"
    echo "  pyenv shell 3.10.12"
    echo "  pip install mujoco numpy vowpalwabbit"
    exit 1
fi

echo "mujoco: OK"
echo ""

# Run training
python3 panda/run_mujoco_panda_bandit.py \
    --xml panda/panda_pick_place_scene.xml \
    --ee-site grip_site \
    --obj-body target_object \
    --table-body table \
    --episodes "$EPISODES" \
    $RENDER_FLAG

echo ""
echo "Training complete!"
