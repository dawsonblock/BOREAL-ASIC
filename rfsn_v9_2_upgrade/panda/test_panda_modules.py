"""
Unit Tests for Panda Modules - RFSN v9.2

Tests cover:
- Action enum and names
- Safety validator constraints
- Context extraction
- Reward computation
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from panda.panda_actions import PandaAction, ACTION_NAMES, get_action_name
from panda.panda_validator import PandaSafetyValidator, PandaSafetyConfig
from panda.panda_context import (
    PandaObs, build_context, context_to_feature_vector,
    quat_angle_error_rad, estimate_grasp_confidence
)
from panda.panda_reward import (
    compute_reward, PandaRewardConfig, is_success
)


# ==================== Test PandaAction ====================

class TestPandaAction:
    """Tests for action enum."""
    
    def test_action_count(self):
        """Verify 8 discrete actions."""
        assert len(PandaAction) == 8
    
    def test_action_values(self):
        """Verify action values are 0-7."""
        assert PandaAction.HOLD == 0
        assert PandaAction.APPROACH_PREGRASP == 1
        assert PandaAction.ALIGN_ORIENTATION == 2
        assert PandaAction.DESCEND == 3
        assert PandaAction.CLOSE_GRIPPER == 4
        assert PandaAction.LIFT == 5
        assert PandaAction.RETREAT == 6
        assert PandaAction.BACKOFF_RESET == 7
    
    def test_action_names(self):
        """Verify all actions have names."""
        for action in PandaAction:
            assert action.value in ACTION_NAMES
            assert ACTION_NAMES[action.value] == action.name
    
    def test_get_action_name(self):
        """Test get_action_name helper."""
        assert get_action_name(0) == "HOLD"
        assert get_action_name(99) == "UNKNOWN_99"


# ==================== Test PandaSafetyValidator ====================

class TestPandaSafetyValidator:
    """Tests for safety validator constraints."""
    
    @pytest.fixture
    def validator(self):
        return PandaSafetyValidator()
    
    @pytest.fixture
    def safe_context(self):
        """Context that passes all safety checks."""
        return {
            "in_collision": False,
            "min_dist_to_obstacle": 0.15,
            "table_clearance": 0.10,
            "joint_vel_norm": 1.0,
            "joint_limit_margin_min": 0.2,
            "ee_to_obj_dist": 0.02,
            "ee_align_err_rad": 0.1,
            "grasp_confidence": 0.8,
        }
    
    def test_hold_always_safe(self, validator, safe_context):
        """HOLD should always be safe."""
        is_safe, _ = validator.validate(PandaAction.HOLD, safe_context)
        assert is_safe
        
        # Even in collision
        unsafe_ctx = {**safe_context, "in_collision": True}
        is_safe, _ = validator.validate(PandaAction.HOLD, unsafe_ctx)
        assert is_safe
    
    def test_collision_blocks_motion(self, validator, safe_context):
        """Collision should block motion actions."""
        unsafe_ctx = {**safe_context, "in_collision": True}
        
        is_safe, reason = validator.validate(PandaAction.LIFT, unsafe_ctx)
        assert not is_safe
        assert "collision" in reason.lower()
        
        # But BACKOFF should be allowed
        is_safe, _ = validator.validate(PandaAction.BACKOFF_RESET, unsafe_ctx)
        assert is_safe
    
    def test_obstacle_distance(self, validator, safe_context):
        """Too close to obstacle should block motion."""
        unsafe_ctx = {**safe_context, "min_dist_to_obstacle": 0.05}
        
        is_safe, reason = validator.validate(PandaAction.APPROACH_PREGRASP, unsafe_ctx)
        assert not is_safe
        assert "obstacle" in reason.lower()
    
    def test_table_clearance(self, validator, safe_context):
        """DESCEND should be blocked if too close to table."""
        unsafe_ctx = {**safe_context, "table_clearance": 0.005}
        
        is_safe, reason = validator.validate(PandaAction.DESCEND, unsafe_ctx)
        assert not is_safe
        assert "table" in reason.lower()
    
    def test_grasp_preconditions(self, validator, safe_context):
        """CLOSE_GRIPPER requires proximity and alignment."""
        # Too far
        far_ctx = {**safe_context, "ee_to_obj_dist": 0.10}
        is_safe, reason = validator.validate(PandaAction.CLOSE_GRIPPER, far_ctx)
        assert not is_safe
        assert "far" in reason.lower()
        
        # Misaligned
        mis_ctx = {**safe_context, "ee_align_err_rad": 1.0}
        is_safe, reason = validator.validate(PandaAction.CLOSE_GRIPPER, mis_ctx)
        assert not is_safe
        assert "misaligned" in reason.lower()
    
    def test_lift_requires_grasp(self, validator, safe_context):
        """LIFT should be blocked without grasp confidence."""
        unsafe_ctx = {**safe_context, "grasp_confidence": 0.3}
        
        is_safe, reason = validator.validate(PandaAction.LIFT, unsafe_ctx)
        assert not is_safe
        assert "grasp_confidence" in reason.lower()
    
    def test_safe_default(self, validator):
        """Safe default should be HOLD."""
        assert validator.get_safe_default() == PandaAction.HOLD


# ==================== Test PandaContext ====================

class TestPandaContext:
    """Tests for context extraction."""
    
    @pytest.fixture
    def sample_obs(self):
        return PandaObs(
            q=np.zeros(7),
            qd=np.array([0.1, 0.2, 0.1, 0.1, 0.1, 0.1, 0.1]),
            ee_pos=np.array([0.5, 0.0, 0.5]),
            ee_quat=np.array([0.0, 0.0, 0.0, 1.0]),
            obj_pos=np.array([0.6, 0.1, 0.3]),
            target_pregrasp_pos=np.array([0.6, 0.1, 0.45]),
            target_grasp_quat=np.array([0.0, 0.0, 0.0, 1.0]),
            min_dist_to_obstacle=0.15,
            table_height_z=0.3,
            in_collision=False,
            gripper_width=0.04,
            gripper_commanded_width=0.04,
            ft_norm=2.5,
            joint_limit_margin_min=0.2,
        )
    
    def test_build_context_keys(self, sample_obs):
        """Verify all expected keys are present."""
        ctx = build_context(sample_obs)
        
        expected_keys = [
            "ee_dx", "ee_dy", "ee_dz", "ee_to_obj_dist",
            "ee_align_err_rad", "min_dist_to_obstacle", "table_clearance",
            "joint_vel_norm", "joint_limit_margin_min", "grasp_confidence",
            "in_collision", "gripper_width", "ft_norm"
        ]
        
        for key in expected_keys:
            assert key in ctx
    
    def test_build_context_values(self, sample_obs):
        """Verify context values are computed correctly."""
        ctx = build_context(sample_obs)
        
        # Check distance
        expected_dist = np.linalg.norm(sample_obs.obj_pos - sample_obs.ee_pos)
        assert abs(ctx["ee_to_obj_dist"] - expected_dist) < 1e-6
        
        # Check table clearance
        expected_clearance = sample_obs.ee_pos[2] - sample_obs.table_height_z
        assert abs(ctx["table_clearance"] - expected_clearance) < 1e-6
        
        # Check joint velocity norm
        expected_vel = np.linalg.norm(sample_obs.qd)
        assert abs(ctx["joint_vel_norm"] - expected_vel) < 1e-6
    
    def test_context_to_feature_vector(self, sample_obs):
        """Verify feature vector has correct shape."""
        ctx = build_context(sample_obs)
        features = context_to_feature_vector(ctx)
        
        assert features.shape == (12,)
        assert features.dtype == np.float32
    
    def test_quat_angle_error(self):
        """Test quaternion angle computation."""
        # Same quaternion = 0 error
        q = np.array([0, 0, 0, 1])
        assert quat_angle_error_rad(q, q) < 1e-6
        
        # 180 degree rotation = pi error
        q1 = np.array([0, 0, 0, 1])
        q2 = np.array([1, 0, 0, 0])  # 180 deg around x
        error = quat_angle_error_rad(q1, q2)
        assert abs(error - np.pi) < 0.1


# ==================== Test PandaReward ====================

class TestPandaReward:
    """Tests for reward computation."""
    
    @pytest.fixture
    def cfg(self):
        return PandaRewardConfig()
    
    @pytest.fixture
    def base_ctx(self):
        return {
            "ee_to_obj_dist": 0.1,
            "ee_align_err_rad": 0.2,
            "grasp_confidence": 0.5,
            "in_collision": False,
            "table_clearance": 0.1,
        }
    
    def test_success_reward(self, cfg, base_ctx):
        """Success should give maximum reward."""
        r = compute_reward(base_ctx, base_ctx, cfg=cfg, success=True, timeout=False, slip=False)
        assert r == cfg.success_reward
    
    def test_collision_penalty(self, cfg, base_ctx):
        """Collision should give maximum penalty."""
        coll_ctx = {**base_ctx, "in_collision": True}
        r = compute_reward(base_ctx, coll_ctx, cfg=cfg, success=False, timeout=False, slip=False)
        assert r == cfg.collision_penalty
    
    def test_timeout_penalty(self, cfg, base_ctx):
        """Timeout should give timeout penalty."""
        r = compute_reward(base_ctx, base_ctx, cfg=cfg, success=False, timeout=True, slip=False)
        assert r == cfg.timeout_penalty
    
    def test_progress_reward(self, cfg, base_ctx):
        """Moving closer should give positive reward."""
        prev = {**base_ctx, "ee_to_obj_dist": 0.2}
        curr = {**base_ctx, "ee_to_obj_dist": 0.1}
        
        r = compute_reward(prev, curr, cfg=cfg, success=False, timeout=False, slip=False)
        assert r > 0  # Positive shaping reward
    
    def test_slip_penalty(self, cfg, base_ctx):
        """Slip should add negative reward."""
        r = compute_reward(base_ctx, base_ctx, cfg=cfg, success=False, timeout=False, slip=True)
        assert cfg.slip_penalty in [r, r - cfg.slip_penalty]  # May have other shaping
    
    def test_is_success(self, base_ctx):
        """Test success detection."""
        # Not success: low confidence
        assert not is_success({**base_ctx, "grasp_confidence": 0.3, "table_clearance": 0.15})
        
        # Not success: not lifted
        assert not is_success({**base_ctx, "grasp_confidence": 0.8, "table_clearance": 0.05})
        
        # Success: good grasp + lifted
        assert is_success({**base_ctx, "grasp_confidence": 0.8, "table_clearance": 0.15})


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
