"""
Panda Safety Validator for RFSN v9.2

Implements hard constraints for Panda robot actions. The validator is consulted
before executing any action and can reject unsafe actions, forcing the bandit
to resample or fall back to a safe default (HOLD).

This follows the "bandits optimize reward, safety must be hard constraints"
principle from v9.2 production enhancements.
"""

from __future__ import annotations

import math
import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# We import the SafetyValidator base class from the integrated VW bandit module.
# This avoids manipulating sys.path at runtime.  If the VW bandit module is
# unavailable, we fall back to a simple abstract base class with the same
# interface.  Importing from rfsn.learning ensures consistency when this
# package is installed as part of the RFSN repo.
try:
    from rfsn.learning.vw_bandit import SafetyValidator  # type: ignore
except Exception:
    class SafetyValidator(ABC):
        """Abstract base for safety validation (local fallback)."""

        @abstractmethod
        def validate(self, action: int, context: dict) -> Tuple[bool, str]:
            """Return (True, reason) if the action is safe, otherwise (False, reason)."""
            pass

        @abstractmethod
        def get_safe_default(self) -> int:
            """Return an action index that is always considered safe."""
            pass

# Import PandaAction from this package rather than a sibling 'panda' module.
from rfsn.panda_bandit.panda_actions import PandaAction


@dataclass
class PandaSafetyConfig:
    """Configuration for safety constraint thresholds."""
    
    # Collision / distance constraints
    min_obstacle_distance_m: float = 0.08      # 8cm minimum clearance
    min_table_clearance_m: float = 0.01        # 1cm above table
    
    # Velocity / acceleration limits
    max_joint_vel_norm: float = 2.5            # rad/s norm bound
    max_joint_pos_margin_rad: float = 0.05     # Near joint limits margin
    
    # Grasp preconditions
    grasp_max_dist_m: float = 0.03             # 3cm max distance to grasp
    grasp_max_angle_rad: float = 0.35          # ~20 degrees max misalignment
    lift_requires_grasp_conf: float = 0.6      # Confidence threshold for lift
    
    # General
    allow_motion_when_colliding: bool = False  # Almost always False


class PandaSafetyValidator(SafetyValidator):
    """
    Hard constraints for Panda robot actions.
    
    Context/state keys expected (from panda_context.build_context):
        - min_dist_to_obstacle: float
        - table_clearance: float
        - joint_vel_norm: float
        - joint_limit_margin_min: float
        - ee_to_obj_dist: float
        - ee_align_err_rad: float
        - grasp_confidence: float
        - in_collision: bool
    
    Example usage:
        >>> validator = PandaSafetyValidator()
        >>> is_safe, reason = validator.validate(PandaAction.DESCEND, context)
        >>> if not is_safe:
        ...     print(f"Blocked: {reason}")
    """
    
    def __init__(self, cfg: Optional[PandaSafetyConfig] = None):
        self.cfg = cfg or PandaSafetyConfig()
    
    def validate(self, action: int, context: Dict) -> Tuple[bool, str]:
        """
        Check whether a proposed action is safe given current context.
        
        Args:
            action: Action index (0-7)
            context: Dict with safety-relevant state
        
        Returns:
            (is_safe, reason): True if safe, else False with explanation
        """
        try:
            act = PandaAction(action)
        except ValueError:
            return False, f"Unknown action index: {action}"
        
        # ===== Constraint A: Never execute motion if in collision =====
        in_collision = bool(context.get("in_collision", False))
        if in_collision and not self.cfg.allow_motion_when_colliding:
            if act not in (PandaAction.HOLD, PandaAction.BACKOFF_RESET):
                return False, "In collision: only HOLD/BACKOFF allowed"
        
        # ===== Constraint B: Obstacle distance for motion actions =====
        min_d = float(context.get("min_dist_to_obstacle", 999.0))
        motion_actions = (
            PandaAction.APPROACH_PREGRASP,
            PandaAction.ALIGN_ORIENTATION,
            PandaAction.DESCEND,
            PandaAction.LIFT,
            PandaAction.RETREAT,
        )
        if act in motion_actions:
            if min_d < self.cfg.min_obstacle_distance_m:
                return False, f"Obstacle too close ({min_d:.3f}m < {self.cfg.min_obstacle_distance_m}m)"
        
        # ===== Constraint C: Table clearance for descending =====
        clearance = float(context.get("table_clearance", 999.0))
        if act == PandaAction.DESCEND:
            if clearance < self.cfg.min_table_clearance_m:
                return False, f"Too close to table ({clearance:.3f}m < {self.cfg.min_table_clearance_m}m)"
        
        # ===== Constraint D: Joint velocity sanity =====
        jv = float(context.get("joint_vel_norm", 0.0))
        velocity_actions = (
            PandaAction.APPROACH_PREGRASP,
            PandaAction.DESCEND,
            PandaAction.LIFT,
            PandaAction.RETREAT,
            PandaAction.BACKOFF_RESET,
        )
        if act in velocity_actions:
            if jv > self.cfg.max_joint_vel_norm:
                return False, f"Joint velocity too high (||qd||={jv:.2f} > {self.cfg.max_joint_vel_norm})"
        
        # ===== Constraint E: Near joint limits =====
        margin = float(context.get("joint_limit_margin_min", 999.0))
        if act in (PandaAction.APPROACH_PREGRASP, PandaAction.DESCEND, PandaAction.LIFT):
            if margin < self.cfg.max_joint_pos_margin_rad:
                return False, f"Near joint limit (margin={margin:.3f} rad < {self.cfg.max_joint_pos_margin_rad} rad)"
        
        # ===== Constraint F: Grasp preconditions =====
        if act == PandaAction.CLOSE_GRIPPER:
            d = float(context.get("ee_to_obj_dist", 999.0))
            ang = float(context.get("ee_align_err_rad", math.pi))
            if d > self.cfg.grasp_max_dist_m:
                return False, f"Object too far to grasp ({d:.3f}m > {self.cfg.grasp_max_dist_m}m)"
            if ang > self.cfg.grasp_max_angle_rad:
                return False, f"Too misaligned to grasp (err={ang:.3f} rad > {self.cfg.grasp_max_angle_rad} rad)"
        
        # ===== Constraint G: Lift requires grasp confidence =====
        if act == PandaAction.LIFT:
            gc = float(context.get("grasp_confidence", 0.0))
            if gc < self.cfg.lift_requires_grasp_conf:
                return False, f"Lift blocked: low grasp_confidence={gc:.2f} < {self.cfg.lift_requires_grasp_conf}"
        
        # All checks passed
        return True, ""
    
    def get_safe_default(self) -> int:
        """
        Return the safest fallback action.
        
        HOLD is always safe - it maintains current position without motion.
        """
        return int(PandaAction.HOLD)
