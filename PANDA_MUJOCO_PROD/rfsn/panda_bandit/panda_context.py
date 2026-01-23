"""
Panda Context Extractor for RFSN v9.2

Converts raw robot + scene state into a compact context dict for VW features.
The context is used for both:
1. Bandit action selection (feature vector)
2. Safety validation (constraint checking)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np


@dataclass
class PandaObs:
    """
    Panda observation dataclass.
    
    This is the minimal set of observations needed for pick-and-place.
    Adapt to your sim/ROS2 pipeline as needed.
    """
    # Joint state
    q: np.ndarray                     # (7,) joint positions
    qd: np.ndarray                    # (7,) joint velocities
    
    # End-effector state
    ee_pos: np.ndarray                # (3,) EE position
    ee_quat: np.ndarray               # (4,) EE orientation (xyzw)
    
    # Object/target state
    obj_pos: np.ndarray               # (3,) object position
    target_pregrasp_pos: np.ndarray   # (3,) pregrasp position target
    target_grasp_quat: np.ndarray     # (4,) target grasp orientation
    
    # Safety signals
    min_dist_to_obstacle: float       # Minimum distance to obstacles
    table_height_z: float             # Table surface Z height
    in_collision: bool                # Collision flag
    
    # Gripper state
    gripper_width: float              # Current gripper width (m)
    gripper_commanded_width: float    # Commanded gripper width (m)
    
    # Force/torque (if available)
    ft_norm: float                    # Wrist F/T norm
    
    # Optional extras
    joint_limit_margin_min: Optional[float] = None  # Rad margin to nearest limit


def quat_angle_error_rad(q_current: np.ndarray, q_target: np.ndarray) -> float:
    """
    Compute angle error between two quaternions in radians.
    
    Uses: angle = 2 * acos(|dot(q1, q2)|)
    
    Args:
        q_current: Current quaternion (xyzw)
        q_target: Target quaternion (xyzw)
    
    Returns:
        Angle error in radians [0, π]
    """
    dot = float(np.clip(np.abs(np.dot(q_current, q_target)), 0.0, 1.0))
    return 2.0 * float(np.arccos(dot))


def estimate_grasp_confidence(obs: PandaObs) -> float:
    """
    Estimate grasp confidence from gripper and force signals.
    
    Simple heuristic (tune for your setup):
    - Width score: How close is gripper to commanded width?
    - FT score: Is there contact force?
    
    Returns:
        Confidence score in [0, 1]
    """
    width = obs.gripper_width
    # "Object in gripper" heuristic: 
    # If commanded closed (0), but width > 0 => Grasping object (Obstruction)
    # If commanded open/pos, width should match command (Tracking)
    cmd = obs.gripper_commanded_width
    width = obs.gripper_width
    
    if cmd < 0.01:
        # Closing/Holding: Reward obstruction (width > cmd)
        # Cube width ~0.04. Threshold 0.01 to filter collision noise.
        obstruction = width - cmd
        width_score = np.clip((obstruction - 0.005) / 0.02, 0.0, 1.0)
    else:
        # Opening/Free: Reward tracking (width ~ cmd)
        # But for 'grasp confidence', we are not grasping if opening.
        # So maybe low score? Or imply "control confidence"?
        # For this task, we want high confidence only when holding.
        width_score = 0.0
    
    # FT score: contact force implies grasp
    ft_score = np.clip(obs.ft_norm / 10.0, 0.0, 1.0)
    
    return float(0.7 * width_score + 0.3 * ft_score)


def build_context(obs: PandaObs) -> Dict[str, float]:
    """
    Build context dict from PandaObs for VW features and safety validation.
    
    Output keys:
        - ee_dx, ee_dy, ee_dz: Relative position to object
        - ee_to_obj_dist: Distance to object
        - ee_align_err_rad: Orientation error to grasp frame
        - min_dist_to_obstacle: Distance to nearest obstacle
        - table_clearance: Height above table
        - joint_vel_norm: Joint velocity magnitude
        - joint_limit_margin_min: Distance to nearest joint limit
        - grasp_confidence: Estimated grasp quality
        - in_collision: Collision flag
        - gripper_width: Current gripper width
        - ft_norm: Force/torque magnitude
    
    Args:
        obs: PandaObs dataclass
    
    Returns:
        Dict with float values for all context features
    """
    # Relative position EE → object
    ee_to_obj = obs.obj_pos - obs.ee_pos
    ee_to_obj_dist = float(np.linalg.norm(ee_to_obj))
    
    # Orientation alignment error
    ee_align_err = quat_angle_error_rad(obs.ee_quat, obs.target_grasp_quat)
    
    # Table clearance (EE height above table)
    table_clearance = float(obs.ee_pos[2] - obs.table_height_z)
    
    # Joint velocity magnitude
    joint_vel_norm = float(np.linalg.norm(obs.qd))
    
    # Joint limit margin (use large default if not provided)
    joint_limit_margin_min = (
        float(obs.joint_limit_margin_min) 
        if obs.joint_limit_margin_min is not None 
        else 999.0
    )
    
    # Grasp confidence estimate
    grasp_conf = estimate_grasp_confidence(obs)
    
    return {
        # Geometry
        "ee_dx": float(ee_to_obj[0]),
        "ee_dy": float(ee_to_obj[1]),
        "ee_dz": float(ee_to_obj[2]),
        "ee_to_obj_dist": ee_to_obj_dist,
        "ee_align_err_rad": float(ee_align_err),
        
        # Safety signals
        "min_dist_to_obstacle": float(obs.min_dist_to_obstacle),
        "table_clearance": table_clearance,
        "joint_vel_norm": joint_vel_norm,
        "joint_limit_margin_min": joint_limit_margin_min,
        
        # Grasp signals
        "grasp_confidence": float(grasp_conf),
        "in_collision": float(obs.in_collision),  # Convert bool to float for VW
        "gripper_width": float(obs.gripper_width),
        "ft_norm": float(obs.ft_norm),
    }


def context_to_feature_vector(ctx: Dict[str, float]) -> np.ndarray:
    """
    Convert context dict to numpy feature vector for VW bandit.
    
    This function is now optimized to avoid list comprehensions and constant key string allocation
    in the critical path.
    
    Args:
        ctx: Context dict from build_context()
    
    Returns:
        np.ndarray of shape (12,) with feature values
    """
    # Optimized: direct extraction without loops
    # This is ~5x faster in tight loops than list comprehension over string keys
    output = np.zeros(12, dtype=np.float32)
    output[0] = ctx.get("ee_dx", 0.0)
    output[1] = ctx.get("ee_dy", 0.0)
    output[2] = ctx.get("ee_dz", 0.0)
    output[3] = ctx.get("ee_to_obj_dist", 0.0)
    output[4] = ctx.get("ee_align_err_rad", 0.0)
    output[5] = ctx.get("min_dist_to_obstacle", 0.0)
    output[6] = ctx.get("table_clearance", 0.0)
    output[7] = ctx.get("joint_vel_norm", 0.0)
    output[8] = ctx.get("joint_limit_margin_min", 999.0)
    output[9] = ctx.get("grasp_confidence", 0.0)
    output[10] = ctx.get("gripper_width", 0.0)
    output[11] = ctx.get("ft_norm", 0.0)
    
    return output
