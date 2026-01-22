"""
Panda Reward Shaping for RFSN v9.2

Reward function designed to work with v9.2's RewardNormalizer + RewardDecayScheduler.
Raw rewards vary in scale, normalization stabilizes learning across:
- Different objects
- Different friction conditions
- Different camera noise
- Different goals
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass
class PandaRewardConfig:
    """Configuration for reward shaping."""
    
    # Terminal rewards
    success_reward: float = 10.0
    collision_penalty: float = -10.0
    timeout_penalty: float = -2.0
    slip_penalty: float = -3.0
    
    # Shaping rewards (per-step)
    progress_scale: float = 2.0      # Scale for distance improvement
    align_scale: float = 1.0         # Scale for alignment improvement
    grasp_bonus: float = 3.0         # Bonus for grasp confidence


def compute_reward(
    prev_ctx: Dict[str, float],
    ctx: Dict[str, float],
    *,
    cfg: PandaRewardConfig,
    success: bool,
    timeout: bool,
    slip: bool,
) -> float:
    """
    Compute reward for a transition.
    
    Terminal conditions take priority:
        - Success: +10 (task complete)
        - Collision: -10 (safety violation)
        - Timeout: -2 (episode limit reached)
    
    Step rewards (shaping):
        - Progress: +2 × (prev_dist - cur_dist) for approaching object
        - Alignment: +1 × (prev_ang - cur_ang) for improving orientation
        - Grasp bonus: +3 × max(0, gc - 0.5) for good grasp confidence
        - Slip: -3 if object slipped during manipulation
    
    Args:
        prev_ctx: Context dict from previous step
        ctx: Context dict from current step
        cfg: Reward configuration
        success: Episode ended in success
        timeout: Episode ended in timeout
        slip: Object slipped during this step
    
    Returns:
        Raw reward value (to be normalized by RewardNormalizer)
    """
    # Terminal conditions (highest priority)
    if success:
        return cfg.success_reward
    
    if ctx.get("in_collision", False):
        return cfg.collision_penalty
    
    if timeout:
        return cfg.timeout_penalty
    
    # Shaping rewards
    r = 0.0
    
    # Progress: Distance to object decreased
    prev_d = float(prev_ctx.get("ee_to_obj_dist", 0.0))
    cur_d = float(ctx.get("ee_to_obj_dist", 0.0))
    r += cfg.progress_scale * (prev_d - cur_d)
    
    # Orientation alignment improved
    prev_a = float(prev_ctx.get("ee_align_err_rad", 0.0))
    cur_a = float(ctx.get("ee_align_err_rad", 0.0))
    r += cfg.align_scale * (prev_a - cur_a)
    
    # Grasp confidence bonus (soft reward above threshold)
    gc = float(ctx.get("grasp_confidence", 0.0))
    r += cfg.grasp_bonus * max(0.0, gc - 0.5)
    
    # Slip penalty
    if slip:
        r += cfg.slip_penalty
    
    return float(r)


def is_success(ctx: Dict[str, float], lift_height_threshold: float = 0.10) -> bool:
    """
    Check if task succeeded based on context.
    
    Simple heuristic: Object is lifted above threshold with good grasp.
    
    Args:
        ctx: Current context
        lift_height_threshold: How high object must be lifted (m)
    
    Returns:
        True if success condition met
    """
    # High grasp confidence + object being lifted
    gc = float(ctx.get("grasp_confidence", 0.0))
    table_clearance = float(ctx.get("table_clearance", 0.0))
    
    return gc > 0.6 and table_clearance > lift_height_threshold
