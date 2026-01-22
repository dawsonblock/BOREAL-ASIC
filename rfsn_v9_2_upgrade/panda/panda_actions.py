"""
Panda Action Space for RFSN v9.2

8 discrete actions for pick-and-place tasks. Each action is semantically
stable across tasks and maps to a deterministic motion primitive.
"""

from enum import IntEnum
from typing import Dict


class PandaAction(IntEnum):
    """
    8 discrete actions for Panda pick-and-place.
    
    Actions are ordered by typical task flow:
    HOLD → APPROACH → ALIGN → DESCEND → CLOSE → LIFT → RETREAT → (BACKOFF if needed)
    """
    HOLD = 0                # Stay still / keep current setpoint
    APPROACH_PREGRASP = 1   # Move EE to pregrasp pose above object
    ALIGN_ORIENTATION = 2   # Rotate EE to target grasp frame
    DESCEND = 3             # Move down along approach vector
    CLOSE_GRIPPER = 4       # Close gripper fingers
    LIFT = 5                # Lift object after confirmed grasp
    RETREAT = 6             # Move to retreat/place position
    BACKOFF_RESET = 7       # Escape pose if stuck or unsafe


# Human-readable action names for logging and debugging
ACTION_NAMES: Dict[int, str] = {a.value: a.name for a in PandaAction}


def get_action_name(action: int) -> str:
    """Get human-readable name for action index."""
    return ACTION_NAMES.get(action, f"UNKNOWN_{action}")
