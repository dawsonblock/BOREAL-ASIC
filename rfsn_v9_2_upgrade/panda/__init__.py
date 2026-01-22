"""
Panda Robot Integration for RFSN v9.2

This package provides:
- PandaAction: 8 discrete actions for pick-and-place
- PandaSafetyValidator: Hard constraints for safe execution
- PandaObs + build_context: Context extraction for VW features
- compute_reward: Reward shaping for bandit learning
- PandaPrimitives: Deterministic motion execution
- MuJoCoPandaIface: MuJoCo robot interface
"""

from panda.panda_actions import PandaAction, ACTION_NAMES
from panda.panda_validator import PandaSafetyValidator, PandaSafetyConfig
from panda.panda_context import PandaObs, build_context
from panda.panda_reward import compute_reward, PandaRewardConfig
from panda.panda_primitives import PandaPrimitives, PrimitiveTargets

__all__ = [
    "PandaAction",
    "ACTION_NAMES",
    "PandaSafetyValidator",
    "PandaSafetyConfig",
    "PandaObs",
    "build_context",
    "compute_reward",
    "PandaRewardConfig",
    "PandaPrimitives",
    "PrimitiveTargets",
]
