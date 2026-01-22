"""
Preconfigured MuJoCo Panda Configurations for RFSN v9.2

This module provides ready-to-use configurations for common Panda setups:
- mujoco_menagerie (official Google DeepMind models)
- Custom pick-place scene

Usage:
    from panda.panda_configs import MENAGERIE_CONFIG, PICK_PLACE_CONFIG
    panda = MuJoCoPandaIface(model, data, PICK_PLACE_CONFIG)
"""

from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np


@dataclass
class MuJoCoPandaConfig:
    """Configuration for MuJoCo Panda interface."""
    
    ee_site: str = "grip_site"
    obj_body: str = "target_object"
    table_body: str = "table"
    gripper_actuator: str = "actuator8"
    
    joint_names: List[str] = field(default_factory=lambda: [
        "joint1", "joint2", "joint3", "joint4", 
        "joint5", "joint6", "joint7"
    ])
    
    ctrl_kp: float = 40.0
    ctrl_kd: float = 4.0
    max_joint_step: float = 0.04
    
    safe_pose_q: Optional[np.ndarray] = None
    
    gripper_open_width: float = 255.0   # actuator8 uses 0-255 scale
    gripper_close_width: float = 0.0


# ==================== Preconfigured Setups ====================

# For our custom pick-place scene (panda_pick_place_scene.xml)
PICK_PLACE_CONFIG = MuJoCoPandaConfig(
    ee_site="grip_site",
    obj_body="target_object",
    table_body="table",
    gripper_actuator="actuator8",
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
    safe_pose_q=np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853]),
    gripper_open_width=255.0,
    gripper_close_width=0.0,
)

# For mujoco_menagerie/franka_emika_panda/scene.xml (no object/table)
MENAGERIE_SCENE_CONFIG = MuJoCoPandaConfig(
    ee_site="grip_site",  # We added this in our custom scene
    obj_body="",          # No object in base scene
    table_body="",        # No table in base scene
    gripper_actuator="actuator8",
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
    safe_pose_q=np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853]),
)


def get_config_for_scene(scene_name: str) -> MuJoCoPandaConfig:
    """
    Get preconfigured settings for a known scene.
    
    Args:
        scene_name: One of "pick_place", "menagerie"
    
    Returns:
        MuJoCoPandaConfig for the scene
    """
    configs = {
        "pick_place": PICK_PLACE_CONFIG,
        "menagerie": MENAGERIE_SCENE_CONFIG,
    }
    
    if scene_name.lower() not in configs:
        raise ValueError(f"Unknown scene: {scene_name}. Available: {list(configs.keys())}")
    
    return configs[scene_name.lower()]
