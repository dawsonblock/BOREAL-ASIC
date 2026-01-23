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

from rfsn.panda_bandit.panda_actions import PandaAction, ACTION_NAMES
from rfsn.panda_bandit.panda_validator import PandaSafetyValidator, PandaSafetyConfig
from rfsn.panda_bandit.panda_context import PandaObs, build_context
from rfsn.panda_bandit.panda_reward import compute_reward, PandaRewardConfig
from rfsn.panda_bandit.panda_primitives import PandaPrimitives, PrimitiveTargets

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
