# 🐼 Panda RFSN v9.2 Integration

> **A robust, safety-constrained reinforcement learning framework for the Franka Emika Panda robot.**

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Python](https://img.shields.io/badge/python-3.10+-blue.svg)
![MuJoCo](https://img.shields.io/badge/mujoco-3.0+-orange.svg)

This repository houses the integration of the **Franka Emika Panda** robotic arm with the **RFSN v9.2 Contextual Bandit** framework. It features a complete physics simulation environment in MuJoCo, deterministic motion primitives, and a safety-validated action space designed for efficient learning of manipulation tasks.

## ✨ Key Features

- **🦾 High-Fidelity Physics**: Custom MuJoCo XML calibrated for realistic grasping (10g mass, 2.0 friction).
- **🛡️ Safety First**: Hard-constrained `PandaSafetyValidator` prevents collisions, joint limit violations, and unsafe motions.
- **🧠 Contextual Bandit Agent**: VW-based agent (`RFSNController`) learning from 12-dimensional state features.
- **⚡ Optimized Performance**: 400ms control loop + 200k step exploration budget for deep policy learning.
- **🔧 Modular Architecture**: Drop-in `panda/` package compatible with existing RFSN infrastructure.

## 🚀 Getting Started

### Prerequisites

- Python 3.10+
- MuJoCo 3.x
- `vowpalwabbit` (VW)

### Installation

```bash
# Clone the repository
git clone https://github.com/dawsonblock/NEW-ROBOT.git
cd NEW-ROBOT

# Install dependencies (assuming standard RFSN env)
pip install mujoco numpy vowpalwabbit
```

### 🏃‍♂️ Running the Training

Train the bandit agent on the Pick-and-Place task:

```bash
# Run headless training (fast)
python panda/run_mujoco_panda_bandit.py \
    --xml panda/panda_pick_place_scene.xml \
    --ee-site grip_site \
    --episodes 1000

# Run with visualization (slower)
python panda/run_mujoco_panda_bandit.py \
    --xml panda/panda_pick_place_scene.xml \
    --ee-site grip_site \
    --episodes 10 \
    --render
```

The model checkpoints will be saved to `./models/`.

### 🧪 Validation

Verify the physics and safety layers manually:

```bash
python panda/demo_pick_place.py
```

## 📂 Architecture

```
panda/
├── panda_actions.py       # 8 discrete actions (Approach, Grasp, Lift...)
├── panda_validator.py     # Safety rules & constraint checking
├── panda_context.py       # Feature extraction (state -> context)
├── panda_reward.py        # Shaped reward function (obstruction-based)
├── mujoco_panda_iface.py  # Low-level simulator interface
└── run_mujoco_panda_bandit.py # Main training loop
```

## 📊 Performance

- **Baseline Penalty**: -2.07 (Timeout)
- **Learning Signal**: -2.11+ (Active exploration/movement)
- **Safety**: 0 Collisions during training

## 📝 License

Distributed under the MIT License. See `LICENSE` for more information.
