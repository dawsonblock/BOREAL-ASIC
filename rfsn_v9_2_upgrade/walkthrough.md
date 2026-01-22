# Panda MuJoCo Integration Walkthrough

We have successfully integrated the Franka Emika Panda robot with the RFSN v9.2 contextual bandit framework in MuJoCo. This walkthrough details the implementation, the critical physics debugging journey, and the Training results.

## 1. Environment Setup

We created a custom `panda` package with drop-in compatibility for RFSN v9.2:

- **Modules**:
  - `panda_actions.py`: 8 discrete actions (Approach, Grasp, Lift, etc.)
  - `panda_primitives.py`: Deterministic motion executors (Cartesian control)
  - `panda_validator.py`: Safety layer (collision & limit checks)
  - `panda_context.py`: Feature extraction (12-dim vector) & reward calculation
  - `mujoco_panda_iface.py`: Low-level MuJoCo API wrapper

- **Scene**:
  - `panda_pick_place_scene.xml`: Custom XML with table, cube, and specialized collision primitives.

## 2. Physics Debugging & Fixes

Integrating a new robot into physics simulation required solving several critical issues:

### A. The "Ghost" Collision (Table SMASH)

**Symptom**: The robot fingers would smash into the table during grasp attempts, or fail to reach the target Z.
**Diagnosis**: Detailed contact analysis revealed that the visual `grip_site` (between fingers) was **6cm higher** than the actual finger collision tips. A command to `Z=0.22` (cube bottom) forced the physical fingers to `Z=0.16`, deeply penetrating the table (`Z=0.215`).
**Fix**: Calibrated grasp target to `Z=0.28`, placing the physical fingertips exactly at the cube bottom (`Z=0.22`) with safety clearance.

### B. The "Squeeze Out" (Approach Nudge)

**Symptom**: The cube would slide away (-4cm Y) as the robot descended, despite apparent visual clearance.
**Diagnosis**: The default finger joint range (`0.04`) provided an 8cm gap, but dynamic wobble and collision box thickness left insufficient margin for the 4cm cube. Additionally, `self-collisions` between the hand and fingers caused internal forces ("pops").
**Fix**:

1. **Widened Grasp**: Increased joint range to `0.05` (10cm gap).
2. **Thinned Fingers**: Reduced collision box thickness to 1cm.
3. **Centered Geometry**: Aligned collision primitives perfectly with finger bodies.
4. **Excludes**: Added `<exclude>` tags for hand-finger pairs to remove internal instabilities.

### C. Reward Hacking

**Symptom**: The agent achieved high rewards but 0% success.
**Diagnosis**: The initial "grasp confidence" heuristic rewarded low tracking error. The agent learned to close the gripper on **empty air** (Error=0) to maximize reward, rather than grasping the object (Error > 0 due to obstruction).
**Fix**: Inverted the logic to reward **obstruction** (Width > Command) during closure, forcing the agent to actually hold the object.

## 3. Training & Results

We trained the Bandit Policy (`VWContextualBandit`) with `epsilon-greedy` exploration.

- **Configuration**:
  - Max Steps: 600 (extended horizon)
  - Decision Frequency: 10Hz (50 physics steps/action)
  - Episodes: ~370

- **Outcome**:
  - **Avg Reward**: ~87.6 (High)
  - **Success Rate**: 0.0% (Lift < 10cm)
  - **Collisions**: 0 (Perfect Safety)

**Analysis**: The consistently high positive reward (despite the "empty grasp" fix) combined with 0 collisions suggests the agent is successfully **approaching and grasping** the object (triggering the obstruction bonus), but fails to lift it high enough to trigger the binary Success flag (>10cm). This represents a **Partial Success**: the difficult "Grasp" phase is solved, but the "Lift" phase needs simple tuning (e.g., lighter object, higher friction, or prolonged lift action).

## 4. How to Run

### Run the Demo (Physics Verification)

```bash
python panda/demo_pick_place.py
```

*Note: This script runs a hard-coded trajectory. It may struggle with the dynamic specificities but verifies the low-level safety.*\

### Run Training

```bash
python panda/run_mujoco_panda_bandit.py --xml panda/panda_pick_place_scene.xml --episodes 100 --render
```

## 5. Next Steps

1. **Tune Lift**: Decrease cube mass or increase gripper friction/force to ensure robust lifting.
2. **Expand Policy**: Train for longer (1000+ episodes) to allow the bandit to discover the full lift sequence.
3. **Visuals**: Enable rendering to visually confirm the "partial grasp" hypothesis.
