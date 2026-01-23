"""
Panda Motion Primitives for RFSN v9.2

Deterministic motion executor layer. The bandit chooses "what to do next",
and primitives execute the motion:

    Bandit → action index → PandaPrimitives.step() → robot motion

This is the correct division of labor:
- Bandit = discrete decision layer
- Primitives = continuous execution layer
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Protocol

import numpy as np

# Import from the integrated Panda bandit package
from rfsn.panda_bandit.panda_actions import PandaAction


class RobotInterface(Protocol):
    """
    Protocol for robot interface implementations.
    
    Implement this for your specific robot backend:
    - MuJoCo simulation
    - ROS2 + MoveIt
    - Cartesian impedance controller
    """
    
    def get_ee_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Get current EE pose (position, quaternion)."""
        ...
    
    def goto_cartesian(self, pos: np.ndarray, quat: np.ndarray) -> None:
        """Move EE to target Cartesian pose."""
        ...
    
    def close_gripper(self) -> None:
        """Close gripper fingers."""
        ...
    
    def open_gripper(self, width: float) -> None:
        """Open gripper to specified width."""
        ...
    
    def hold(self) -> None:
        """Maintain current position."""
        ...
    
    def backoff_to_safe_pose(self) -> None:
        """Move to safe home pose."""
        ...


@dataclass
class PrimitiveTargets:
    """Target poses for motion primitives."""
    
    pregrasp_pos: np.ndarray      # (3,) pregrasp position
    grasp_quat: np.ndarray        # (4,) target grasp orientation
    retreat_pos: np.ndarray       # (3,) retreat/place position
    lift_delta_z: float = 0.12    # How high to lift (meters)
    descend_delta_z: float = 0.04 # How far to descend per action (meters)


class PandaPrimitives:
    """
    Deterministic motion primitive executor for Panda.
    
    Maps discrete actions to robot motions:
        HOLD → maintain current joint setpoint
        APPROACH_PREGRASP → Cartesian servo to pregrasp pose
        ALIGN_ORIENTATION → rotate EE to target quaternion
        DESCEND → move down by descend_delta_z
        CLOSE_GRIPPER → command gripper to close
        LIFT → move up by lift_delta_z
        RETREAT → move to retreat position
        BACKOFF_RESET → return to safe home pose
    
    Example:
        >>> robot = MuJoCoPandaIface(model, data, cfg)
        >>> prims = PandaPrimitives(robot)
        >>> targets = PrimitiveTargets(pregrasp_pos, grasp_quat, retreat_pos)
        >>> info = prims.step(PandaAction.APPROACH_PREGRASP, targets)
    """
    
    def __init__(self, robot_iface: Any):
        """
        Initialize primitives executor.
        
        Args:
            robot_iface: Robot interface (MuJoCoPandaIface or similar)
        """
        self.robot = robot_iface
    
    def step(self, action: int, targets: PrimitiveTargets, duration: int = 1) -> Dict[str, Any]:
        """
        Execute one action step.
        
        Args:
            action: Action index (0-7)
            targets: Target poses for primitives
            duration: Number of execution steps (chunking). Default 1.
                      Higher = smoother motion, fewer bandit calls.
        
        Returns:
            Dict with execution info:
                - done: bool, episode termination
                - success: bool, task completed
                - slip: bool, object slipped
        """
        try:
            act = PandaAction(action)
        except ValueError:
            # Unknown action, do nothing
            self.robot.hold()
            return {"done": False, "success": False, "slip": False}
        
        # Scaling factor based on duration to make primitives less jumpy
        scale = 1.0 / max(1, duration / 5.0)  # Heuristic damping

        if act == PandaAction.HOLD:
            self.robot.hold()
            return {"done": False}
        
        if act == PandaAction.APPROACH_PREGRASP:
            self.robot.goto_cartesian(targets.pregrasp_pos, targets.grasp_quat)
            return {"done": False}
        
        if act == PandaAction.ALIGN_ORIENTATION:
            cur_pos, _ = self.robot.get_ee_pose()
            self.robot.goto_cartesian(cur_pos, targets.grasp_quat)
            return {"done": False}
        
        if act == PandaAction.DESCEND:
            cur_pos, cur_q = self.robot.get_ee_pose()
            descend_pos = cur_pos.copy()
            # If duration > 1, take smaller steps? 
            # Actually, primitive is a "target setter".
            # The physics loop interpolates.
            descend_pos[2] -= targets.descend_delta_z
            self.robot.goto_cartesian(descend_pos, cur_q)
            return {"done": False}
        
        if act == PandaAction.CLOSE_GRIPPER:
            self.robot.close_gripper()
            return {"done": False}
        
        if act == PandaAction.LIFT:
            cur_pos, cur_q = self.robot.get_ee_pose()
            lift_pos = cur_pos.copy()
            lift_pos[2] += targets.lift_delta_z
            self.robot.goto_cartesian(lift_pos, cur_q)
            return {"done": False}
        
        if act == PandaAction.RETREAT:
            self.robot.goto_cartesian(targets.retreat_pos, targets.grasp_quat)
            return {"done": False}
        
        if act == PandaAction.BACKOFF_RESET:
            self.robot.backoff_to_safe_pose()
            return {"done": False}
        
        # Fallback: hold
        self.robot.hold()
        return {"done": False}
