"""
MuJoCo Panda Interface for RFSN v9.2

Low-level interface for controlling Panda in MuJoCo simulation.
Provides deterministic execution of motion primitives.

Configurable via MuJoCoPandaConfig for different XML files:
- mujoco_menagerie/franka_emika_panda
- robosuite panda
- custom XMLs
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

try:
    import mujoco
except ImportError:
    mujoco = None


@dataclass
class MuJoCoPandaConfig:
    """
    Configuration for MuJoCo Panda interface.
    
    Customize these names to match your XML file:
    - mujoco_menagerie uses: link0, link1-7, hand, finger joints
    - robosuite uses: robot0_, panda_
    - custom XMLs vary
    """
    # Site/body/actuator names (change to match your XML)
    ee_site: str = "attachment_site"          # End-effector site name
    obj_body: str = "target_object"           # Object body name
    table_body: str = "table"                 # Table body name
    gripper_actuator: str = "actuator8"       # Gripper actuator name
    
    # Joint names pattern (Panda has 7 arm joints)
    joint_names: List[str] = field(default_factory=lambda: [
        "joint1", "joint2", "joint3", "joint4", 
        "joint5", "joint6", "joint7"
    ])
    
    # Control parameters
    ctrl_kp: float = 40.0                     # Position control gain
    ctrl_kd: float = 4.0                      # Velocity damping
    max_joint_step: float = 0.04              # Max rad per step (safety clamp)
    
    # Safe pose (7D joint configuration)
    safe_pose_q: Optional[np.ndarray] = None  # Override with your safe pose
    
    # Gripper parameters
    gripper_open_width: float = 0.04          # Open gripper width (m)
    gripper_close_width: float = 0.0          # Closed gripper width (m)


class MuJoCoPandaIface:
    """
    Deterministic low-level executor for MuJoCo Panda.
    
    Assumptions:
    - You control Panda joints via position control in data.ctrl
    - Gripper is one actuator (or map accordingly)
    - EE pose comes from a named site
    
    Example:
        >>> model = mujoco.MjModel.from_xml_path("panda.xml")
        >>> data = mujoco.MjData(model)
        >>> cfg = MuJoCoPandaConfig(ee_site="ee_site")
        >>> panda = MuJoCoPandaIface(model, data, cfg)
        >>> pos, quat = panda.get_ee_pose()
    """
    
    def __init__(
        self, 
        model: "mujoco.MjModel", 
        data: "mujoco.MjData", 
        cfg: MuJoCoPandaConfig
    ):
        if mujoco is None:
            raise RuntimeError("mujoco python package not available. Install with: pip install mujoco")
        
        self.model = model
        self.data = data
        self.cfg = cfg
        
        # Find EE site
        self.ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, cfg.ee_site)
        if self.ee_site_id < 0:
            # Try to find any site containing "ee", "end", or "attachment"
            for i in range(model.nsite):
                site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
                if site_name and any(x in site_name.lower() for x in ["ee", "end", "attach", "hand"]):
                    self.ee_site_id = i
                    break
        if self.ee_site_id < 0:
            raise ValueError(f"EE site '{cfg.ee_site}' not found in model. Available sites: {self._list_sites()}")
        
        # Find object body (optional, may not exist in all scenes)
        self.obj_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, cfg.obj_body)
        
        # Find table body (optional)
        self.table_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, cfg.table_body)
        
        # Find gripper actuator (optional)
        # Support dual actuators (left/right) automatically
        self.gripper_act_ids = []
        
        # 1. Try configured name
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, cfg.gripper_actuator)
        if gid >= 0:
            self.gripper_act_ids.append(gid)
            
        # 2. Try standard panda names if specific one not found or checks for commonly used separate actuators
        for name in ["panda_gripper_left", "panda_gripper_right", "panda_finger_left", "panda_finger_right"]:
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if gid >= 0 and gid not in self.gripper_act_ids:
                self.gripper_act_ids.append(gid)
                
        self.gripper_act_id = self.gripper_act_ids[0] if self.gripper_act_ids else -1
        
        # Identify 7 arm joints
        self.joint_ids = []
        for jn in cfg.joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                self.joint_ids.append(jid)
        
        # Fallback: find first 7 hinge joints if named joints not found
        if len(self.joint_ids) != 7:
            self.joint_ids = [
                j for j in range(model.njnt) 
                if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
            ][:7]
        
        if len(self.joint_ids) < 7:
            raise ValueError(f"Could not identify 7 Panda arm joints. Found: {len(self.joint_ids)}")
        
        # qpos/qvel indices for joints
        self.qpos_adr = [model.jnt_qposadr[j] for j in self.joint_ids]
        self.qvel_adr = [model.jnt_dofadr[j] for j in self.joint_ids]
        
        # Safe pose (default to current or zeros)
        if cfg.safe_pose_q is None:
            self.safe_pose_q = self.get_q().copy()
        else:
            self.safe_pose_q = np.asarray(cfg.safe_pose_q, dtype=np.float64)
    
    def _list_sites(self) -> List[str]:
        """List all sites in model for debugging."""
        sites = []
        for i in range(self.model.nsite):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
            if name:
                sites.append(name)
        return sites
    
    # ==================== State Getters ====================
    
    def get_q(self) -> np.ndarray:
        """Get current joint positions (7,)."""
        return np.array([self.data.qpos[a] for a in self.qpos_adr], dtype=np.float64)
    
    def get_qd(self) -> np.ndarray:
        """Get current joint velocities (7,)."""
        return np.array([self.data.qvel[a] for a in self.qvel_adr], dtype=np.float64)
    
    def get_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get end-effector pose.
        
        Returns:
            (position, quaternion): (3,) and (4,) arrays, quat is xyzw
        """
        pos = self.data.site_xpos[self.ee_site_id].copy()
        xmat = self.data.site_xmat[self.ee_site_id].reshape(3, 3).copy()
        quat = _mat_to_quat_xyzw(xmat)
        return pos, quat
    
    def get_obj_pos(self) -> np.ndarray:
        """Get object position (3,). Returns zeros if object not found."""
        if self.obj_body_id < 0:
            return np.zeros(3)
        return self.data.xpos[self.obj_body_id].copy()
    
    def get_table_height(self) -> float:
        """Get table surface Z height. Returns 0 if table not found."""
        if self.table_body_id < 0:
            return 0.0
        return float(self.data.xpos[self.table_body_id][2])
    
    def get_gripper_width(self) -> float:
        """Get current gripper width."""
        if self.gripper_act_id is None or self.gripper_act_id < 0:
            return self.cfg.gripper_open_width
        return float(self.data.ctrl[self.gripper_act_id])
    
    # ==================== Motion Primitives ====================
    
    def hold(self) -> None:
        """Maintain current position (set ctrl targets = current q)."""
        q = self.get_q()
        self._set_arm_ctrl(q)
    
    def backoff_to_safe_pose(self) -> None:
        """Move to safe home pose."""
        self._set_arm_ctrl(self.safe_pose_q)
    
    def close_gripper(self) -> None:
        """Close gripper."""
        if not hasattr(self, "gripper_act_ids") or not self.gripper_act_ids:
            return
        for gid in self.gripper_act_ids:
            self.data.ctrl[gid] = self.cfg.gripper_close_width
    
    def open_gripper(self, width: Optional[float] = None) -> None:
        """Open gripper to specified width."""
        if not hasattr(self, "gripper_act_ids") or not self.gripper_act_ids:
            return
        w = width if width is not None else self.cfg.gripper_open_width
        for gid in self.gripper_act_ids:
            self.data.ctrl[gid] = float(w)
    
    def goto_joint_targets(self, q_target: np.ndarray) -> None:
        """Move toward joint targets with step limiting."""
        q = self.get_q()
        dq = np.clip(
            q_target - q, 
            -self.cfg.max_joint_step, 
            self.cfg.max_joint_step
        )
        self._set_arm_ctrl(q + dq)
    
    def goto_cartesian(self, target_pos: np.ndarray, target_quat: np.ndarray) -> None:
        """
        Move EE toward target Cartesian pose using damped least squares IK.
        
        Position-only for robustness. Orientation handled separately via ALIGN.
        """
        q_new = _dls_ik_position_only(
            model=self.model,
            data=self.data,
            site_id=self.ee_site_id,
            target_pos=target_pos,
            qpos_adr=self.qpos_adr,
            joint_ids=self.joint_ids,
            iters=20,
            damp=1e-2,
            step=0.6,
        )
        self.goto_joint_targets(q_new)
    
    def _set_arm_ctrl(self, q_cmd: np.ndarray) -> None:
        """Set arm control targets (first 7 ctrl slots)."""
        for i, q_val in enumerate(q_cmd[:7]):
            if i < len(self.data.ctrl):
                self.data.ctrl[i] = q_val


def _mat_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to quaternion (xyzw convention)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def _dls_ik_position_only(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    site_id: int,
    target_pos: np.ndarray,
    qpos_adr: List[int],
    joint_ids: List[int],
    iters: int = 20,
    damp: float = 1e-2,
    step: float = 0.6,
) -> np.ndarray:
    """
    Damped least squares IK for position only.
    
    Solves: dq = J^T (J J^T + λI)^-1 err
    """
    q = np.array([data.qpos[a] for a in qpos_adr], dtype=np.float64)
    
    for _ in range(iters):
        # Current EE position
        cur = data.site_xpos[site_id].copy()
        err = target_pos - cur
        
        if np.linalg.norm(err) < 1e-3:
            break
        
        # Compute Jacobian (3 x nv) for site position
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        
        # Extract columns for our 7 DOFs
        dof_adr = [model.jnt_dofadr[jid] for jid in joint_ids]
        J = jacp[:, dof_adr]  # (3, 7)
        
        # DLS solve
        JJt = J @ J.T
        dq = J.T @ np.linalg.solve(JJt + (damp * np.eye(3)), err)
        
        q = q + step * dq
        
        # Write back to data.qpos and forward kinematics
        for i, a in enumerate(qpos_adr):
            data.qpos[a] = q[i]
        mujoco.mj_forward(model, data)
    
    return q
