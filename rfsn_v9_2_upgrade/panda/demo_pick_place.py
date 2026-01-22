#!/usr/bin/env python3
"""
Scripted Pick-and-Place Demo for Panda

This demonstrates the pick-and-place task working without the bandit,
using hand-coded Cartesian waypoints. Once this works, we can train
the bandit to replicate it.

Usage:
    python3.10 panda/demo_pick_place.py
"""

import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    import mujoco
except ImportError:
    print("ERROR: Install mujoco: pip install mujoco")
    sys.exit(1)

from panda.mujoco_panda_iface import MuJoCoPandaIface, MuJoCoPandaConfig


def run_demo():
    print("Loading scene...")
    model = mujoco.MjModel.from_xml_path("panda/panda_pick_place_scene.xml")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    
    cfg = MuJoCoPandaConfig(
        ee_site="grip_site", 
        obj_body="target_object", 
        table_body="table",
        gripper_open_width=255.0,
        gripper_close_width=0.0,
    )
    panda = MuJoCoPandaIface(model, data, cfg)
    
    # Get cube position
    cube_pos = panda.get_obj_pos().copy()
    table_z = panda.get_table_height() + 0.015  # table surface
    
    print(f"Cube at: {cube_pos}")
    print(f"Table Z: {table_z}")
    
    # Define waypoints for pick-and-place
    # Contact analysis shows finger tips are ~6cm below grip_site.
    # To place tips at 0.22 (cube bottom), grip_site must be at 0.22 + 0.06 = 0.28.
    pregrasp = cube_pos.copy()
    pregrasp[2] = 0.35
    
    grasp_pos = cube_pos.copy()
    grasp_pos[2] = 0.27  # Corrected sweet spot (0.28 too high, 0.255 too low)
    
    lift_pos = grasp_pos.copy()
    lift_pos[2] = 0.45
    
    place_pos = cube_pos.copy()
    place_pos[0] += 0.15
    place_pos[2] = 0.30
    
    # Use current orientation as natural grasp orientation
    _, start_quat = panda.get_ee_pose()
    
    # helper: rotate quaternion 90 deg around Z
    # q_z90 = [0.707, 0, 0, 0.707]
    def quat_mult(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    qz90 = np.array([0.7071068, 0, 0, 0.7071068])
    grasp_quat = quat_mult(start_quat, qz90)
    
    def step_physics(n=10):
        for _ in range(n):
            mujoco.mj_step(model, data)
    
    def move_to(target_pos, steps=100):
        """Move EE to target position using IK"""
        for _ in range(steps):
            panda.goto_cartesian(target_pos, grasp_quat)
            step_physics(5)
        return panda.get_ee_pose()[0]
    
    # ========== EXECUTE PICK AND PLACE ==========
    
    print("\n=== Step 1: Open gripper ===")
    panda.open_gripper()
    step_physics(50)
    
    print("\n=== Step 2: Move to pregrasp ===")
    ee = move_to(pregrasp, steps=150)
    print(f"EE at: {ee}, target: {pregrasp}")
    
    print("\n=== Step 3: Descend to grasp ===")
    # Slow descent to avoid dynamic collision wobble
    for _ in range(200):
        panda.goto_cartesian(grasp_pos, grasp_quat)
        step_physics(5)
        # Check if cube moved
        curr_cube = panda.get_obj_pos()
        dist = np.linalg.norm(curr_cube - cube_pos)
        if dist > 0.01:
            print(f"  WARNING: Cube moved! Dist={dist:.3f} at EE Z={panda.get_ee_pose()[0][2]:.3f}")
            break
            
    ee = panda.get_ee_pose()[0]
    print(f"EE at: {ee}, target: {grasp_pos}, cube: {panda.get_obj_pos()}")
    
    print("\n=== Step 4: Close gripper ===")
    panda.close_gripper()
    # Step physics and monitor contacts
    for i in range(100):
        mujoco.mj_step(model, data)
        if i % 10 == 0:
            print(f"Step {i}: Width={panda.get_gripper_width():.4f}, Contacts={data.ncon}")
            for c_i in range(data.ncon):
                c = data.contact[c_i]
                g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or f"id{c.geom1}"
                g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or f"id{c.geom2}"
                # Only print interesting contacts
                if "finger" in str(g1) or "finger" in str(g2) or "cube" in str(g1) or "cube" in str(g2):
                    print(f"  Contact: {g1} <-> {g2} (dist={c.dist:.4f})")
    
    print(f"Gripper width: {panda.get_gripper_width()}")
    
    print("\n=== Step 5: Lift ===")
    ee = move_to(lift_pos, steps=100)
    cube_after = panda.get_obj_pos()
    print(f"EE at: {ee}")
    print(f"Cube at: {cube_after}")
    
    # Check if we successfully grasped
    cube_lifted = cube_after[2] > cube_pos[2] + 0.05
    if cube_lifted:
        print("\n✅ SUCCESS! Cube was lifted!")
    else:
        print("\n❌ FAILED - Cube not lifted")
        print(f"   Cube started at Z={cube_pos[2]:.3f}, now at Z={cube_after[2]:.3f}")
    
    print("\n=== Step 6: Move to place position ===")
    place_above = place_pos.copy()
    place_above[2] += 0.10
    ee = move_to(place_above, steps=100)
    
    print("\n=== Step 7: Lower to place ===")
    ee = move_to(place_pos, steps=80)
    
    print("\n=== Step 8: Open gripper to release ===")
    panda.open_gripper()
    step_physics(50)
    
    print("\n=== Step 9: Retreat ===")
    retreat_pos = place_pos.copy()
    retreat_pos[2] += 0.15
    ee = move_to(retreat_pos, steps=80)
    
    # Final check
    final_cube = panda.get_obj_pos()
    print(f"\n=== FINAL STATE ===")
    print(f"Cube final position: {final_cube}")
    print(f"Cube X displacement: {final_cube[0] - cube_pos[0]:.3f}m")
    
    if abs(final_cube[0] - place_pos[0]) < 0.05:
        print("✅ Cube placed at target location!")
    else:
        print("❌ Cube not at target location")


if __name__ == "__main__":
    run_demo()
