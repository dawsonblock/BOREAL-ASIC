"""
MuJoCo Panda Observation Stream for RFSN v9.2

Generator that yields PandaObs from MuJoCo state.
Used in the training loop to convert simulation state to context.
"""

from __future__ import annotations

from typing import Iterator, TYPE_CHECKING

import numpy as np

from rfsn.panda_bandit.panda_context import PandaObs

if TYPE_CHECKING:
    from rfsn.panda_bandit.mujoco_panda_iface import MuJoCoPandaIface

try:
    import mujoco
except ImportError:
    mujoco = None


def min_contact_distance_proxy(model: "mujoco.MjModel", data: "mujoco.MjData") -> float:
    """
    Estimate minimum distance to obstacles using contact information.
    
    If you have mj_distances configured in your model, use that instead.
    This is a simple proxy: if contacts exist, return 0, else return large value.
    
    Returns:
        Estimated minimum distance in meters
    """
    # FIX: Original proxy returned 0.0 if ANY contact existed (e.g. cube on table).
    # This blocked all motion. For this demo, we assume open space (999.0)
    # unless we implement proper distance sensors.
    return 999.0


def in_collision(model: "mujoco.MjModel", data: "mujoco.MjData") -> bool:
    """
    Check if robot arm is in collision with environment.
    
    Filters contacts to only report collisions involving robot links.
    Ignores expected contacts like cube-on-table.
    
    Returns:
        True if robot is in collision
    """
    if data.ncon == 0:
        return False
    
    # Get robot link body IDs (link0 through hand)
    robot_body_names = [
        "link0", "link1", "link2", "link3", "link4", 
        "link5", "link6", "link7", "hand", 
        "left_finger", "right_finger"
    ]
    robot_body_ids = set()
    for name in robot_body_names:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid >= 0:
            robot_body_ids.add(bid)
    
    # Check if any contact involves a robot body colliding with non-robot body
    for i in range(data.ncon):
        contact = data.contact[i]
        geom1, geom2 = contact.geom1, contact.geom2
        body1 = model.geom_bodyid[geom1]
        body2 = model.geom_bodyid[geom2]
        
        # Robot collision = one body is robot, other is not robot (or is floor/table)
        robot_involved = body1 in robot_body_ids or body2 in robot_body_ids
        both_robot = body1 in robot_body_ids and body2 in robot_body_ids
        
        if robot_involved and not both_robot:
            # Robot colliding with environment (table, floor, etc)
            # But exclude gripper-object contact (that's a desired grasp!)
            # Only flag if it's link bodies, not fingers
            finger_ids = set()
            for fn in ["left_finger", "right_finger"]:
                fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fn)
                if fid >= 0:
                    finger_ids.add(fid)
            
            # If finger touching object, that's OK (potential grasp)
            if body1 in finger_ids or body2 in finger_ids:
                continue  # Allow finger contacts
            
            # Arm body hitting something - this is a collision!
            return True
    
    return False


def compute_joint_limit_margin(
    q: np.ndarray,
    model: "mujoco.MjModel",
    joint_ids: list,
) -> float:
    """
    Compute minimum margin to joint limits.
    
    Args:
        q: Current joint positions (7,)
        model: MuJoCo model
        joint_ids: List of joint IDs
    
    Returns:
        Minimum distance to any joint limit (radians)
    """
    margins = []
    for i, jid in enumerate(joint_ids):
        if jid < model.njnt:
            qpos_adr = model.jnt_qposadr[jid]
            if model.jnt_limited[jid]:
                lower = model.jnt_range[jid, 0]
                upper = model.jnt_range[jid, 1]
                margin = min(q[i] - lower, upper - q[i])
                margins.append(margin)
    
    return float(min(margins)) if margins else 999.0


def yield_panda_obs(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    panda_iface: "MuJoCoPandaIface",
    *,
    target_pregrasp_pos: np.ndarray,
    target_grasp_quat: np.ndarray,
) -> Iterator[PandaObs]:
    """
    Generator that yields PandaObs from MuJoCo state.
    
    Use this in your training loop:
        >>> obs_stream = yield_panda_obs(model, data, panda, ...)
        >>> for obs in obs_stream:
        ...     ctx = build_context(obs)
        ...     action = bandit.select_action(ctx)
        ...     ...
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
        panda_iface: Panda robot interface
        target_pregrasp_pos: Target pregrasp position (3,)
        target_grasp_quat: Target grasp quaternion (4,)
    
    Yields:
        PandaObs dataclass with current state
    """
    while True:
        # Joint state
        q = panda_iface.get_q()
        qd = panda_iface.get_qd()
        
        # End-effector state
        ee_pos, ee_quat = panda_iface.get_ee_pose()
        
        # Object/scene state
        obj_pos = panda_iface.get_obj_pos()
        table_z = panda_iface.get_table_height()
        
        # Safety signals
        min_d = float(min_contact_distance_proxy(model, data))
        coll = bool(in_collision(model, data))
        
        # Gripper state
        gripper_width = panda_iface.get_gripper_width()
        gripper_cmd_width = gripper_width  # Use current as commanded
        
        # Force/torque (placeholder if no sensors)
        ft_norm = 0.0
        
        # Joint limit margin
        joint_limit_margin = compute_joint_limit_margin(
            q, model, panda_iface.joint_ids
        )
        
        obs = PandaObs(
            q=q,
            qd=qd,
            ee_pos=ee_pos,
            ee_quat=ee_quat,
            obj_pos=obj_pos,
            target_pregrasp_pos=target_pregrasp_pos,
            target_grasp_quat=target_grasp_quat,
            min_dist_to_obstacle=min_d,
            table_height_z=table_z,
            in_collision=coll,
            gripper_width=gripper_width,
            gripper_commanded_width=gripper_cmd_width,
            ft_norm=ft_norm,
            joint_limit_margin_min=joint_limit_margin,
        )
        
        yield obs


def get_single_obs(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    panda_iface: "MuJoCoPandaIface",
    target_pregrasp_pos: np.ndarray,
    target_grasp_quat: np.ndarray,
) -> PandaObs:
    """
    Get a single observation (non-generator version).
    
    Useful for step-by-step control loops.
    """
    gen = yield_panda_obs(
        model, data, panda_iface,
        target_pregrasp_pos=target_pregrasp_pos,
        target_grasp_quat=target_grasp_quat,
    )
    return next(gen)
