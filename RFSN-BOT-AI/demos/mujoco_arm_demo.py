#!/usr/bin/env python3
"""
RFSN Panda Arm - ROBUST SMOOTH Pick & Place Demo
==========================================
Combines Robust logic (grasp check) with Smooth motion (interpolation)
to avoid torque saturation and ensure reliable execution.

Run: uv run python demos/mujoco_arm_demo.py
"""

import mujoco
import numpy as np
import time
import glfw
import os
from datetime import datetime

try:
    import imageio
    HAS_IMAGEIO = True
except ImportError:
    HAS_IMAGEIO = False
    print("Note: Install imageio for video recording: pip install imageio imageio-ffmpeg")

MODEL_PATH = "panda_table_cube.xml"

# Reliable PD gains
KP = np.array([500.0, 500.0, 500.0, 500.0, 300.0, 200.0, 100.0])
KD = np.array([50.0, 50.0, 50.0, 50.0, 30.0, 20.0, 10.0])

# Waypoints
WAYPOINTS = [
    # name, joints, gripper_open_bool, duration(s)
    ("Home", np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]), True, 2.0),
    ("Above Cube", np.array([0.3, -0.5, 0.0, -1.7, 0.0, 1.2, 0.785]), True, 2.0),
    ("Lower to Cube", np.array([0.3, -0.15, 0.0, -1.3, 0.0, 0.85, 0.785]), True, 1.5),
    ("Grasp", np.array([0.3, -0.15, 0.0, -1.3, 0.0, 0.85, 0.785]), False, 1.0),
    ("Lift", np.array([0.3, -0.6, 0.0, -2.0, 0.0, 1.4, 0.785]), False, 1.5),
    ("Move", np.array([-0.4, -0.5, 0.0, -1.8, 0.0, 1.3, 0.785]), False, 2.0),
    ("Lower Place", np.array([-0.4, -0.1, 0.0, -1.25, 0.0, 0.9, 0.785]), False, 1.5),
    ("Release", np.array([-0.4, -0.1, 0.0, -1.25, 0.0, 0.9, 0.785]), True, 1.0),
    ("Retract", np.array([-0.4, -0.5, 0.0, -1.8, 0.0, 1.3, 0.785]), True, 1.5),
]

class RobustDemo:
    def __init__(self):
        print("\n" + "=" * 60)
        print("🤖 RFSN PANDA - SMOOTH & ROBUST DEMO")
        print("=" * 60)
        
        # Load model
        import os
        path = MODEL_PATH
        if not os.path.exists(path):
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", MODEL_PATH)
        
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        
        # Setup
        self.home_q = WAYPOINTS[0][1]
        self.data.qpos[:7] = self.home_q
        mujoco.mj_forward(self.model, self.data)
        
        # State
        self.target_q = self.home_q.copy()
        self.current_ref_q = self.home_q.copy() # For interpolation
        self.gripper_target = 0.04
        self.running = False
        self.stage = 0
        self.stage_start_time = 0.0
        
        # Keyboard control state
        self.paused = False
        self.manual_mode = False
        self.recording = False
        self.record_frames = []
        self.manual_target = np.array([0.3, 0.0, 0.6])  # XYZ target for manual mode
        self.key_state = {}  # Track held keys
        
        # IDs - Cube 1 (red)
        self.cube_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self.cube_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        # IDs - Cube 2 (blue)
        self.cube2_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cube2")
        self.cube2_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "cube2_geom")
        # Finger IDs
        self.left_pad = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "panda_finger_left_geom")
        self.right_pad = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "panda_finger_right_geom")
        
        # Stacking state
        self.cubes_stacked = 0  # 0=none, 1=first placed, 2=stack complete
        self.stack_base_pos = np.array([0.25, 0.0, 0.435])  # Fixed stack location (center-right)
        self.current_cube_idx = 0  # 0=red, 1=blue

    def check_grasp(self):
        """Check if BOTH fingers are in contact with the current target cube."""
        # Select cube based on which one we're picking
        if self.current_cube_idx == 0:
            cube_geom_id = self.cube_geom
        else:
            cube_geom_id = self.cube2_geom
        
        left_finger_id = self.left_pad
        right_finger_id = self.right_pad
        
        left_contact = False
        right_contact = False
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            g1, g2 = contact.geom1, contact.geom2
            
            # Check left finger-cube contact
            if (g1 == cube_geom_id and g2 == left_finger_id) or \
               (g2 == cube_geom_id and g1 == left_finger_id):
                left_contact = True
            
            # Check right finger-cube contact
            if (g1 == cube_geom_id and g2 == right_finger_id) or \
               (g2 == cube_geom_id and g1 == right_finger_id):
                right_contact = True
        
        # Require BOTH fingers touching cube
        return left_contact and right_contact
    
    def compute_approach_quat(self, cube_pos):
        """
        Compute gripper quaternion for optimal approach to cube.
        Gripper points down and rotates around Z to approach from robot base side.
        """
        # Vector from cube to robot base (at origin)
        dx = -cube_pos[0]  # Robot base is at origin
        dy = -cube_pos[1]
        
        # Angle to rotate gripper around Z axis (approach from base side)
        approach_angle = np.arctan2(dy, dx)
        
        # Quaternion: first rotate 180 deg about X (point down), then rotate about Z
        # q_down = [0, 1, 0, 0] (180 deg about X)
        # q_z = [cos(a/2), 0, 0, sin(a/2)] (rotation about Z)
        # q_final = q_z * q_down
        
        half_angle = approach_angle / 2
        q_z = np.array([np.cos(half_angle), 0, 0, np.sin(half_angle)])
        q_down = np.array([0.0, 1.0, 0.0, 0.0])
        
        return self._quat_mul(q_z, q_down)

    def solve_ik(self, target_pos, target_quat=None):
        """
        Solves Inverse Kinematics to find joint angles for a reach target.
        Includes orientation control to keep gripper pointing straight down.
        """
        # Create a dedicated IK data instance if not exists
        if not hasattr(self, 'ik_data'):
            self.ik_data = mujoco.MjData(self.model)
        
        # Default orientation: gripper pointing straight down
        # Quaternion for Z-axis pointing down: 180 deg rotation about X
        if target_quat is None:
            target_quat = np.array([0.0, 1.0, 0.0, 0.0])  # Gripper facing down
        
        # Sync IK data with current state (warm start)
        self.ik_data.qpos[:7] = self.data.qpos[:7]
        self.ik_data.qpos[7:9] = self.data.qpos[7:9]
        mujoco.mj_forward(self.model, self.ik_data)
        
        hand_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "panda_hand")
        
        # Optimization loop (Damped Least Squares with orientation)
        for _ in range(100):  # More iterations for orientation
            # 1. Position Error
            curr_pos = self.ik_data.xpos[hand_id]
            err_pos = target_pos - curr_pos
            
            # 2. Orientation Error (axis-angle representation)
            curr_quat = self.ik_data.xquat[hand_id]
            # Compute quaternion error: q_err = q_target * q_curr^-1
            # For axis-angle error, extract the rotation axis * angle from difference
            quat_conj = curr_quat.copy()
            quat_conj[1:] *= -1  # Conjugate
            # q_err = q_target * conj(q_curr)
            q_err = self._quat_mul(target_quat, quat_conj)
            # Convert to axis-angle (the vector part * 2 approximates the error)
            err_rot = 2.0 * q_err[1:4]  # Small angle approximation
            if q_err[0] < 0:
                err_rot = -err_rot  # Handle quaternion double-cover
            
            # Combined error
            error = np.concatenate([err_pos, err_rot * 0.3])  # Weight rotation less
            
            if np.linalg.norm(error[:3]) < 0.005 and np.linalg.norm(error[3:]) < 0.1:
                break

            # 2. Jacobian (position and rotation)
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jac(self.model, self.ik_data, jacp, jacr, curr_pos, hand_id)
            
            # Full 6D Jacobian for arm joints
            J = np.vstack([jacp[:, :7], jacr[:, :7]])
            
            # 3. Solve dq using damped least squares
            damping = 0.05
            J_T = J.T
            dq = J_T @ np.linalg.solve(J @ J_T + damping * np.eye(6), error)
            
            # Update q with step size
            self.ik_data.qpos[:7] += dq * 0.3
            
            # Clamp to joint limits
            for i in range(7):
                lo, hi = self.model.jnt_range[i]
                self.ik_data.qpos[i] = np.clip(self.ik_data.qpos[i], lo, hi)
            
            mujoco.mj_forward(self.model, self.ik_data)
            
        return self.ik_data.qpos[:7].copy()
    
    def _quat_mul(self, q1, q2):
        """Multiply two quaternions [w, x, y, z]."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def _save_recording(self):
        """Save recorded frames as MP4 video."""
        if not self.record_frames:
            print("⚠ No frames to save")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"demo_recording_{timestamp}.mp4"
        
        print(f"💾 Saving {len(self.record_frames)} frames to {filename}...")
        writer = imageio.get_writer(filename, fps=30, codec='libx264')
        for frame in self.record_frames:
            writer.append_data(frame)
        writer.close()
        print(f"✅ Recording saved: {filename}")
        self.record_frames = []

    def run(self):
        if not glfw.init(): return
        window = glfw.create_window(1280, 960, "Dynamic IK Demo", None, None)
        glfw.make_context_current(window)
        glfw.swap_interval(1)
        
        scene = mujoco.MjvScene(self.model, maxgeom=1000)
        context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation, cam.distance, cam.lookat = 135, -20, 2.0, [0.1, 0, 0.4]
        opt = mujoco.MjvOption()
        
        # State Machine
        STATES = ["INIT", "SCAN", "HOVER", "LOWER", "GRASP", "LIFT", "MOVE_AWAY", "LOWER_PLACE", "RELEASE", "HOME", "RESET"]
        current_state_idx = 0
        
        print("\n🚀 Starting Dynamic IK Loop...")
        print("\n📋 CONTROLS:")
        print("  SPACE - Pause/Resume")
        print("  M     - Toggle Manual Mode")
        print("  R     - Start/Stop Recording")
        print("  G     - Toggle Gripper (manual mode)")
        print("  WASD  - Move target XY (manual mode)")
        print("  Q/E   - Move target Z (manual mode)")
        print("  Arrow keys - Rotate camera")
        print("  ESC   - Quit\n")
        
        self.running = True
        self.stage_start_time = time.time()
        self.start_q = self.home_q.copy()
        
        # Keyboard callback
        def key_callback(window, key, scancode, action, mods):
            if action == glfw.PRESS:
                if key == glfw.KEY_SPACE:
                    self.paused = not self.paused
                    print(f"{'⏸ PAUSED' if self.paused else '▶ RESUMED'}")
                elif key == glfw.KEY_M:
                    self.manual_mode = not self.manual_mode
                    print(f"{'🎮 MANUAL MODE' if self.manual_mode else '🤖 AUTO MODE'}")
                elif key == glfw.KEY_R and HAS_IMAGEIO:
                    self.recording = not self.recording
                    if self.recording:
                        self.record_frames = []
                        print("🔴 Recording started...")
                    else:
                        self._save_recording()
                elif key == glfw.KEY_G:
                    self.gripper_target = 0.04 if self.gripper_target < 0.02 else 0.0
                    print(f"Gripper: {'OPEN' if self.gripper_target > 0.02 else 'CLOSED'}")
                elif key == glfw.KEY_ESCAPE:
                    glfw.set_window_should_close(window, True)
            
            # Track key state for continuous movement
            if action == glfw.PRESS:
                self.key_state[key] = True
            elif action == glfw.RELEASE:
                self.key_state[key] = False
        
        glfw.set_key_callback(window, key_callback)
        
        # Get Reference Orientation (Down) from "Above Cube" waypoint (pre-calculated hack)
        # Or just use the Home orientation? Home is diagonal.
        # Let's assume the solver will find a path that maintains reasonable orientation if we start from a good pose.
        
        real_start_time = time.time()
        sim_time = 0.0
        
        target_pos_world = np.array([0.0, 0.0, 0.0])
        
        while not glfw.window_should_close(window):
            state = STATES[current_state_idx]
            
            # Only skip STATE LOGIC if paused - still render and handle input
            skip_logic = self.paused or self.manual_mode
            
            # Logic Update (skip if paused or manual mode)
            if self.running and not skip_logic:
                t = time.time() - self.stage_start_time
                done = False
                
                # Check for state start
                if not hasattr(self, 'state_initialized') or not self.state_initialized:
                     # FIRST FRAME OF STATE
                     self.state_initialized = True
                     
                     if state == "INIT":
                         pass
                     elif state == "SCAN":
                         pass
                     elif state == "HOVER":
                         # Target: Cube + 20cm Z with optimal approach orientation
                         tgt = target_pos_world + np.array([0.0, 0.0, 0.2])
                         approach_quat = self.compute_approach_quat(target_pos_world)
                         self.target_goal_q = self.solve_ik(tgt, approach_quat)
                         self.start_q = self.data.qpos[:7].copy()
                     elif state == "LOWER":
                         # EE site is 6cm below finger tips. Cube is 1.5cm tall.
                         # Target: 8cm above cube so fingers surround cube center
                         tgt = target_pos_world + np.array([0.0, 0.0, 0.08])
                         approach_quat = self.compute_approach_quat(target_pos_world)
                         self.target_goal_q = self.solve_ik(tgt, approach_quat)
                         self.start_q = self.data.qpos[:7].copy()
                     elif state == "GRASP":
                         pass
                     elif state == "LIFT":
                         tgt = target_pos_world + np.array([0.0, 0.0, 0.3])
                         approach_quat = self.compute_approach_quat(target_pos_world)
                         self.target_goal_q = self.solve_ik(tgt, approach_quat)
                         self.start_q = self.data.qpos[:7].copy()
                     elif state == "MOVE_AWAY":
                         # Move to placement position (within reach: ~0.25m from base)
                         tgt = np.array([0.25, 0.20, 0.60])
                         self.target_goal_q = self.solve_ik(tgt)
                         self.start_q = self.data.qpos[:7].copy()
                     elif state == "LOWER_PLACE":
                         # Lower to table height
                         tgt = np.array([0.25, 0.20, 0.50])
                         self.target_goal_q = self.solve_ik(tgt)
                         self.start_q = self.data.qpos[:7].copy()
                     elif state == "RELEASE":
                         pass
                     elif state == "RESET":
                         pass
                
                # Continuous Logic
                if state == "INIT":
                    self.target_q = self.home_q
                    self.gripper_target = 0.04
                    if t > 1.0: done = True
                    
                elif state == "SCAN":
                    # Look for current target cube
                    if self.current_cube_idx == 0:
                        cube_body = self.cube_body
                        cube_name = "🔴 Red Cube"
                        freejoint = "cube_freejoint"
                        reset_pos = np.array([0.3, 0.0, 0.47, 1.0, 0.0, 0.0, 0.0])
                    else:
                        cube_body = self.cube2_body
                        cube_name = "🔵 Blue Cube"
                        freejoint = "cube2_freejoint"
                        reset_pos = np.array([-0.2, 0.15, 0.47, 1.0, 0.0, 0.0, 0.0])
                    
                    cube_pos = self.data.xpos[cube_body]
                    print(f"👀 Targeting {cube_name} at: [{cube_pos[0]:.2f}, {cube_pos[1]:.2f}, {cube_pos[2]:.2f}]")
                    
                    # Check if cube is reachable (within ~35cm of robot base)
                    dist_from_base = np.sqrt(cube_pos[0]**2 + cube_pos[1]**2)
                    unreachable = cube_pos[2] < 0.2 or dist_from_base > 0.35 or dist_from_base < 0.15
                    
                    if unreachable:
                        print(f"⚠ {cube_name} out of reach! Resetting position...")
                        self.data.joint(freejoint).qpos = reset_pos
                        self.data.joint(freejoint).qvel = np.zeros(6)
                        mujoco.mj_forward(self.model, self.data)
                        
                        target_pos_world = reset_pos[:3].copy()
                        done = True
                    else:
                        target_pos_world = cube_pos.copy()
                        done = True
                    
                elif state == "HOVER":
                    # Target: Cube + 20cm Z (Safe Hover)
                    tgt = target_pos_world + np.array([0.0, 0.0, 0.2])
                    # Solve IK
                    if t == 0.0:
                         ik_q = self.solve_ik(tgt)
                         self.target_goal_q = ik_q
                    
                    duration = 2.0
                    prog = min(1.0, t / duration)
                    alpha = prog * prog * (3 - 2 * prog)
                    self.current_ref_q = self.start_q + alpha * (self.target_goal_q - self.start_q)
                    self.target_q = self.current_ref_q
                    if prog >= 1.0: done = True
                    
                elif state == "LOWER":
                    # Target: Cube Center + Offset for Gripper Length
                    # Finger Center Z is ~0.06 from Wrist.
                    # Cube Z is ~0.445. Wrist Target => 0.445 + 0.06 = 0.505.
                    tgt = target_pos_world + np.array([0.0, 0.0, 0.06])
                    
                    if t == 0.0:
                         ik_q = self.solve_ik(tgt)
                         self.target_goal_q = ik_q
                         self.start_q = self.data.qpos[:7].copy()
                    
                    duration = 1.5
                    prog = min(1.0, t / duration)
                    alpha = prog * prog * (3 - 2 * prog)
                    self.current_ref_q = self.start_q + alpha * (self.target_goal_q - self.start_q)
                    self.target_q = self.current_ref_q
                    if prog >= 1.0: done = True

                elif state == "GRASP":
                    self.gripper_target = 0.0
                    if t > 0.8:  # Wait longer for close
                        # Check both contact AND gripper width
                        # Cube is 3cm wide - if gripper width > 0.5cm, we're holding something
                        gripper_width = abs(self.data.qpos[8] - self.data.qpos[7])
                        has_contact = self.check_grasp()
                        has_width = gripper_width > 0.005 and gripper_width < 0.05
                        
                        if has_contact or has_width:
                            print(f"✅ Grasped! (contact={has_contact}, width={gripper_width*100:.1f}cm)")
                            done = True
                        else:
                            if t > 3.0:  # More time - abort and go back to HOME
                                print("⚠ Grasp failed! Returning to home...")
                                self.gripper_target = 0.04  # Open gripper
                                current_state_idx = 9  # Jump to HOME
                                self.state_initialized = False
                                self.stage_start_time = time.time()
                                print(f"👉 State: {STATES[current_state_idx]}")
                                continue  # Skip normal state advance
                                
                elif state == "LIFT":
                    # Select current cube body for tracking
                    curr_cube_body = self.cube_body if self.current_cube_idx == 0 else self.cube2_body
                    
                    # Lift straight up (Cube + 30cm)
                    tgt = target_pos_world + np.array([0.0, 0.0, 0.3])
                    if not hasattr(self, '_lift_initialized') or not self._lift_initialized:
                         ik_q = self.solve_ik(tgt)
                         self.target_goal_q = ik_q
                         self.start_q = self.data.qpos[:7].copy()
                         self.pre_lift_cube_z = self.data.xpos[curr_cube_body][2]
                         self._lift_initialized = True
                    
                    duration = 1.5
                    prog = min(1.0, t / duration)
                    self.current_ref_q = self.start_q + prog * (self.target_goal_q - self.start_q)
                    self.target_q = self.current_ref_q
                    
                    if prog >= 1.0:
                        # POST-LIFT CHECK: Did the cube actually lift?
                        cube_z = self.data.xpos[curr_cube_body][2]
                        lift_amount = cube_z - self.pre_lift_cube_z
                        self._lift_initialized = False  # Reset for next attempt
                        
                        if lift_amount < 0.05:  # Less than 5cm lift = failed
                            print(f"⚠ Cube dropped! (lifted only {lift_amount*100:.1f}cm) Retrying...")
                            self.gripper_target = 0.04  # Open gripper
                            current_state_idx = 1  # Go back to SCAN
                            self.state_initialized = False
                            self.stage_start_time = time.time()
                            print(f"👉 State: {STATES[current_state_idx]}")
                            continue
                        else:
                            print(f"✅ Cube lifted {lift_amount*100:.1f}cm!")
                            done = True

                elif state == "MOVE_AWAY":
                    # Move to stack position (center-front of table)
                    # Use stack_base_pos XY, high Z for arc
                    stack_xy = self.stack_base_pos[:2]
                    tgt = np.array([stack_xy[0], stack_xy[1], 0.75])  # High arc
                    if t == 0.0:
                         ik_q = self.solve_ik(tgt)
                         self.target_goal_q = ik_q
                         self.start_q = self.data.qpos[:7].copy()
                    
                    duration = 2.0
                    prog = min(1.0, t / duration)
                    self.current_ref_q = self.start_q + prog * (self.target_goal_q - self.start_q)
                    self.target_q = self.current_ref_q
                    if prog >= 1.0:
                        done = True

                elif state == "LOWER_PLACE":
                    # Keep gripper closed during placement
                    self.gripper_target = 0.0
                    
                    # Lower to stack position - height depends on cubes already stacked
                    # Cube is 3cm tall (0.015 half-size), table at 0.42
                    # First cube: place at table level (Z ~0.50 for gripper)
                    # Second cube: place on top of first (+0.035m higher)
                    stack_z = 0.50 if self.cubes_stacked == 0 else 0.535
                    stack_xy = self.stack_base_pos[:2]
                    tgt = np.array([stack_xy[0], stack_xy[1], stack_z])
                    
                    if t == 0.0:
                         ik_q = self.solve_ik(tgt)
                         self.target_goal_q = ik_q
                         self.start_q = self.data.qpos[:7].copy()
                         print(f"📦 Placing {'base' if self.cubes_stacked==0 else 'on stack'} at Z={stack_z:.2f}")
                    
                    duration = 2.5 # Slow and gentle
                    prog = min(1.0, t / duration)
                    self.current_ref_q = self.start_q + prog * (self.target_goal_q - self.start_q)
                    self.target_q = self.current_ref_q
                    
                    # Settle for 0.5s before releasing
                    if prog >= 1.0:
                        if t > duration + 0.5:
                            done = True

                elif state == "RELEASE":
                    self.gripper_target = 0.04
                    self.soft_grip = True # Soft release
                    
                    # Lift while releasing to avoid smacking
                    tgt = target_pos_world + np.array([0.0, 0.0, 0.2])
                    if t == 0.0:
                         ik_q = self.solve_ik(tgt)
                         self.target_goal_q = ik_q
                         self.start_q = self.data.qpos[:7].copy()
                    
                    duration = 1.5
                    prog = min(1.0, t / duration)
                    self.current_ref_q = self.start_q + prog * (self.target_goal_q - self.start_q)
                    self.target_q = self.current_ref_q
                    
                    if prog >= 1.0: done = True

                elif state == "HOME":
                     # Go to Home
                     self.target_goal_q = self.home_q
                     if t == 0.0:
                          self.start_q = self.data.qpos[:7].copy()
                     
                     duration = 2.0
                     prog = min(1.0, t / duration)
                     self.current_ref_q = self.start_q + prog * (self.target_goal_q - self.start_q)
                     self.target_q = self.current_ref_q
                     if prog >= 1.0: done = True

                elif state == "RESET":
                    # Increment cubes stacked counter
                    self.cubes_stacked += 1
                    
                    if self.cubes_stacked >= 2:
                        # Stack complete!
                        print("🏆 STACK COMPLETE! Two cubes stacked successfully!")
                        print("🔄 Starting new stack cycle...")
                        # Reset for new stack
                        self.cubes_stacked = 0
                        self.current_cube_idx = 0
                        # Reset cube positions for next cycle
                        self.data.joint("cube_freejoint").qpos = np.array([0.3, 0.0, 0.47, 1.0, 0.0, 0.0, 0.0])
                        self.data.joint("cube_freejoint").qvel = np.zeros(6)
                        self.data.joint("cube2_freejoint").qpos = np.array([-0.2, 0.15, 0.47, 1.0, 0.0, 0.0, 0.0])
                        self.data.joint("cube2_freejoint").qvel = np.zeros(6)
                        mujoco.mj_forward(self.model, self.data)
                    else:
                        # Move to next cube
                        self.current_cube_idx = 1
                        cube_name = "🔵 Blue Cube" if self.current_cube_idx == 1 else "🔴 Red Cube"
                        print(f"📦 First cube placed. Now targeting {cube_name}...")
                    
                    # Loop back to SCAN
                    current_state_idx = 0
                    self.stage_start_time = time.time()
                    self.state_initialized = False
                    continue

                if done:
                    current_state_idx += 1
                    self.stage_start_time = time.time()
                    self.state_initialized = False # RESET FLAG
                    self.soft_grip = False # Reset soft grip
                    print(f"👉 State: {STATES[current_state_idx]}")


            # 2. Physics Stepping
            wall_time = time.time() - real_start_time
            max_steps = 20
            steps = 0
            while sim_time < wall_time and steps < max_steps:
                # Position actuators: send target joint positions directly
                # The MuJoCo position actuators have built-in PD control (kp=2000, kv=100)
                self.data.ctrl[:7] = self.target_q
                
                # Gripper: position actuators expect position targets
                # OPEN: Left=0.035, Right=-0.035 (fingers apart)
                # CLOSE: Left=-0.01, Right=0.01 (fingers together)
                if self.gripper_target > 0.02:  # OPEN
                    self.data.ctrl[7] = 0.035
                    self.data.ctrl[8] = -0.035
                else:  # CLOSE - tighter grip
                    self.data.ctrl[7] = -0.025
                    self.data.ctrl[8] = 0.025
                
                mujoco.mj_step(self.model, self.data)
                sim_time += self.model.opt.timestep
                steps += 1

            # 3. Handle camera controls (arrow keys)
            if self.key_state.get(glfw.KEY_LEFT, False):
                cam.azimuth -= 2
            if self.key_state.get(glfw.KEY_RIGHT, False):
                cam.azimuth += 2  
            if self.key_state.get(glfw.KEY_UP, False):
                cam.elevation = min(cam.elevation + 2, 90)
            if self.key_state.get(glfw.KEY_DOWN, False):
                cam.elevation = max(cam.elevation - 2, -90)
            
            # 4. Handle manual mode target movement
            if self.manual_mode:
                move_speed = 0.01
                if self.key_state.get(glfw.KEY_W, False):
                    self.manual_target[0] += move_speed
                if self.key_state.get(glfw.KEY_S, False):
                    self.manual_target[0] -= move_speed
                if self.key_state.get(glfw.KEY_A, False):
                    self.manual_target[1] += move_speed
                if self.key_state.get(glfw.KEY_D, False):
                    self.manual_target[1] -= move_speed
                if self.key_state.get(glfw.KEY_Q, False):
                    self.manual_target[2] += move_speed
                if self.key_state.get(glfw.KEY_E, False):
                    self.manual_target[2] -= move_speed
                
                # Update IK target for manual mode
                self.target_q = self.solve_ik(self.manual_target)

            # 5. Render
            viewport = mujoco.MjrRect(0, 0, *glfw.get_framebuffer_size(window))
            mujoco.mjv_updateScene(self.model, self.data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(viewport, scene, context)
            
            # Overlay with status
            mode_str = "MANUAL" if self.manual_mode else "AUTO"
            rec_str = " [REC]" if self.recording else ""
            pause_str = " [PAUSED]" if self.paused else ""
            msg = f"State: {state} | Mode: {mode_str}{rec_str}{pause_str}"
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, viewport, msg, None, context)
            
            # 6. Capture frame for recording
            if self.recording and HAS_IMAGEIO:
                width, height = glfw.get_framebuffer_size(window)
                pixels = np.zeros((height, width, 3), dtype=np.uint8)
                mujoco.mjr_readPixels(pixels, None, viewport, context)
                self.record_frames.append(np.flipud(pixels))
            
            glfw.swap_buffers(window)
            glfw.poll_events()
            
        glfw.terminate()

if __name__ == "__main__":
    RobustDemo().run()
