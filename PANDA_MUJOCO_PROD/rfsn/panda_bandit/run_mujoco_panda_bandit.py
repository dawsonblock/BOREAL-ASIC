#!/usr/bin/env python3
"""
MuJoCo Panda Bandit Training Demo for RFSN v9.2

Runs the complete training loop:
1. Load Panda XML into MuJoCo
2. Initialize RFSNController with 8 actions, 12-dim context
3. Run episodes with select_action_with_safety() + PandaSafetyValidator
4. Step physics and update bandit with normalized rewards
5. Optional viewer for visualization

Usage:
    # Headless training
    python panda/run_mujoco_panda_bandit.py --xml path/to/panda.xml --episodes 100

    # Optimized visualization
    python panda/run_mujoco_panda_bandit.py --xml path/to/panda.xml --episodes 10 --render --render-every 10

    # With specific config
    python panda/run_mujoco_panda_bandit.py --xml path/to/panda.xml --epsilon 0.3 --lr 0.05
"""

from __future__ import annotations

import argparse
import logging
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

# Imports updated for the integrated RFSN package structure.
# Instead of relying on sys.path hacks to import local modules, import
# bandit and Panda helpers directly from the rfsn package. The
# corresponding modules live under rfsn.learning and rfsn.panda_bandit.

try:
    from rfsn.learning.vw_bandit import (
        VWConfig,
        VWContextualBandit,
        RewardNormalizer,
        RewardDecayScheduler,
    )
    logging.getLogger(__name__).info("Loaded Vowpal Wabbit bandit")
except ImportError:
    from rfsn.learning.fallback_bandit import (
        VWConfig,
        VWContextualBandit,
        RewardNormalizer,
        RewardDecayScheduler,
    )
    logging.getLogger(__name__).warning("Could not import Vowpal Wabbit - using safe fallback (Random/No-Op)")

from rfsn.panda_bandit.panda_actions import PandaAction, ACTION_NAMES
from rfsn.panda_bandit.panda_validator import PandaSafetyValidator, PandaSafetyConfig
from rfsn.panda_bandit.panda_context import build_context, context_to_feature_vector
from rfsn.panda_bandit.panda_reward import compute_reward, PandaRewardConfig, is_success
from rfsn.panda_bandit.panda_primitives import PandaPrimitives, PrimitiveTargets
from rfsn.panda_bandit.mujoco_panda_iface import MuJoCoPandaIface, MuJoCoPandaConfig
from rfsn.panda_bandit.mujoco_panda_obs import get_single_obs

try:
    import mujoco
    import mujoco.viewer
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    mujoco = None

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


@dataclass
class EpisodeConfig:
    """Episode configuration."""
    max_steps: int = 600
    steps_per_action: int = 200  # Physics steps per bandit action
    success_height: float = 0.10  # Lift height for success (m)


@dataclass
class TrainingConfig:
    """Training configuration."""
    n_episodes: int = 100
    epsilon: float = 0.2
    learning_rate: float = 0.1
    exploration_budget: Optional[int] = 200000
    min_epsilon: float = 0.01
    checkpoint_freq: int = 100
    render: bool = False
    render_every: int = 10  # Render every N physics steps

class PandaBanditRunner:
    """
    Training loop runner for Panda + RFSN v9.2 bandit.
    
    Handles:
    - Episode loops with safety-validated action selection
    - Reward normalization and decay
    - Metrics tracking
    - Optional visualization
    """
    
    def __init__(
        self,
        model: "mujoco.MjModel",
        data: "mujoco.MjData",
        panda_iface: MuJoCoPandaIface,
        bandit: VWContextualBandit,
        training_cfg: TrainingConfig,
        episode_cfg: EpisodeConfig,
    ):
        self.model = model
        self.data = data
        self.panda = panda_iface
        self.bandit = bandit
        self.training_cfg = training_cfg
        self.episode_cfg = episode_cfg
        
        # Components
        self.validator = PandaSafetyValidator()
        self.prims = PandaPrimitives(panda_iface)
        self.reward_cfg = PandaRewardConfig()
        self.reward_normalizer = RewardNormalizer(alpha=0.01)
        self.reward_decay = RewardDecayScheduler(half_life=10000)
        
        # Metrics
        self.episode_rewards = []
        self.successes = 0
        self.collisions = 0
        self.total_steps = 0
    
    def setup_targets(self) -> PrimitiveTargets:
        """
        Setup target poses for the current episode.
        
        Default: pregrasp above object, retreat to the side.
        """
        obj_pos = self.panda.get_obj_pos()
        _, grasp_quat = self.panda.get_ee_pose()
        
        # Pregrasp: 12cm above object
        pregrasp = obj_pos.copy()
        pregrasp[2] += 0.12
        
        # Retreat: to the side
        retreat = pregrasp.copy()
        retreat[0] -= 0.15
        
        return PrimitiveTargets(
            pregrasp_pos=pregrasp,
            grasp_quat=grasp_quat,
            retreat_pos=retreat,
            lift_delta_z=0.12,
            descend_delta_z=0.04,
        )
    
    def run_episode(self, targets: PrimitiveTargets, viewer=None) -> dict:
        """
        Run a single episode.
        
        Returns:
            Dict with episode metrics
        """
        prev_ctx = None
        ep_reward = 0.0
        success = False
        collision = False
        
        # Configuration for optimization
        DECIDE_EVERY = 20  # Decision frequency (action every N physics macro-steps?) 
        # Actually, self.episode_cfg.steps_per_action controls the "duration" of a primitive.
        # But we can also implement a meta-controller loop here.
        
        # Let's trust steps_per_action as the chunk size for now (it's 200!).
        # That means we already decide only every 200 physics steps.
        # This IS the optimizations requested (chunked primitives).
        
        # But we can optimize the FEATURE construction and VW update.
        
        prev_features = None  # Cache features to avoid rebuilding if context hasn't changed enough?
                              # No, context changes after physical motion.

        for step in range(self.episode_cfg.max_steps):
            # Get observation (Fast MuJoCo read)
            obs = get_single_obs(
                self.model, self.data, self.panda,
                targets.pregrasp_pos, targets.grasp_quat
            )
            
            # Build context (Python overhead)
            ctx = build_context(obs)
            
            if prev_ctx is None:
                prev_ctx = ctx
            
            # --- DECISION BLOCK ---
            # Safety-validated action selection
            action = self._select_safe_action(ctx)
            
            # Execute primitive (Sets control targets)
            # We pass duration=steps_per_action to hint the primitive layer about chunking
            info = self.prims.step(action, targets, duration=self.episode_cfg.steps_per_action)
            
            # --- PHYSICS CHUNK ---
            # Run physics for N steps without Python interruption
            for p_step in range(self.episode_cfg.steps_per_action):
                mujoco.mj_step(self.model, self.data)
                
                # Optimized Rendering Loop
                render_freq = getattr(self.training_cfg, 'render_every', 10)
                if viewer is not None and (p_step % render_freq == 0):
                    viewer.sync()
            
            # --- LEARNING BLOCK ---
            # Detect termination conditions
            success = is_success(ctx, self.episode_cfg.success_height)
            timeout = (step >= self.episode_cfg.max_steps - 1)
            slip = False 
            collision = bool(ctx.get("in_collision", False))
            
            # Compute reward
            raw_reward = compute_reward(
                prev_ctx, ctx,
                cfg=self.reward_cfg,
                success=success,
                timeout=timeout,
                slip=slip,
            )
            
            # Normalize and decay
            decayed = self.reward_decay.apply(raw_reward)
            normalized = self.reward_normalizer.normalize(decayed)
            self.reward_decay.step()
            
            # Update bandit
            # Optimize: Only convert context to features ONCE
            features = context_to_feature_vector(ctx)
            self.bandit.update(features, int(action), float(normalized))
            
            ep_reward += raw_reward
            self.total_steps += 1
            prev_ctx = ctx
            
            if success or collision:
                break
        
        return {
            "reward": ep_reward,
            "success": success,
            "collision": collision,

            "steps": step + 1,
        }
    
    def _select_safe_action(self, ctx: dict) -> int:
        """Select action with safety validation and resampling."""
        features = context_to_feature_vector(ctx)
        
        for attempt in range(5):
            action, _ = self.bandit.select_action(features)
            is_safe, reason = self.validator.validate(action, ctx)
            if is_safe:
                return action
            logger.debug(f"Unsafe action {ACTION_NAMES[action]}: {reason}")
        
        # Fall back to safe default
        return self.validator.get_safe_default()
    
    def train(self) -> dict:
        """
        Run full training loop.
        
        Returns:
            Training summary dict
        """
        logger.info(f"Starting training: {self.training_cfg.n_episodes} episodes")
        
        viewer = None
        if self.training_cfg.render and MUJOCO_AVAILABLE:
            try:
                viewer = mujoco.viewer.launch_passive(self.model, self.data)
            except RuntimeError as e:
                if "mjpython" in str(e):
                    logger.warning(
                        "macOS requires 'mjpython' for visualization. "
                        "Running headless. To enable rendering, run:\n"
                        "  mjpython panda/run_mujoco_panda_bandit.py --xml ... --render"
                    )
                else:
                    raise
        
        try:
            for ep in range(self.training_cfg.n_episodes):
                # Reset environment
                mujoco.mj_resetData(self.model, self.data)
                mujoco.mj_forward(self.model, self.data)
                self.panda.open_gripper()
                
                # Setup targets for this episode
                targets = self.setup_targets()
                
                # Run episode
                result = self.run_episode(targets, viewer)
                
                # Track metrics
                self.episode_rewards.append(result["reward"])
                if result["success"]:
                    self.successes += 1
                if result["collision"]:
                    self.collisions += 1
                
                # Logging
                if (ep + 1) % self.training_cfg.checkpoint_freq == 0:
                    success_rate = self.successes / (ep + 1) * 100
                    avg_reward = np.mean(self.episode_rewards[-self.training_cfg.checkpoint_freq:])
                    logger.info(
                        f"Episode {ep + 1}/{self.training_cfg.n_episodes} | "
                        f"Avg Reward: {avg_reward:.2f} | "
                        f"Success Rate: {success_rate:.1f}% | "
                        f"Collisions: {self.collisions}"
                    )
        finally:
            if viewer is not None:
                viewer.close()
        
        return {
            "total_episodes": self.training_cfg.n_episodes,
            "total_steps": self.total_steps,
            "successes": self.successes,
            "collisions": self.collisions,
            "success_rate": self.successes / self.training_cfg.n_episodes,
            "avg_reward": float(np.mean(self.episode_rewards)),
            "final_10_avg": float(np.mean(self.episode_rewards[-10:])),
        }


def create_vw_config(cfg: TrainingConfig) -> VWConfig:
    """Create VWConfig for Panda bandit."""
    return VWConfig(
        n_actions=len(PandaAction),  # 8 actions
        context_dim=12,              # 12 context features
        learning_rate=cfg.learning_rate,
        epsilon=cfg.epsilon,
        exploration_strategy="epsilon",
        bits=18,
        save_frequency=100,
        exploration_budget=cfg.exploration_budget,
        min_epsilon=cfg.min_epsilon,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Panda MuJoCo Bandit Training for RFSN v9.2"
    )
    parser.add_argument(
        "--xml", type=str, required=True,
        help="Path to Panda MuJoCo XML file"
    )
    parser.add_argument(
        "--episodes", type=int, default=100,
        help="Number of training episodes"
    )
    parser.add_argument(
        "--epsilon", type=float, default=0.2,
        help="Exploration epsilon"
    )
    parser.add_argument(
        "--lr", type=float, default=0.1,
        help="Learning rate"
    )
    parser.add_argument(
        "--render", action="store_true",
        help="Enable visualization"
    )
    parser.add_argument(
        "--render-every", type=int, default=10,
        help="Render every N physics steps (default: 10). Set to 1 for smooth demo, higher for speed."
    )
    parser.add_argument(
        "--max-steps", type=int, default=300,
        help="Max steps per episode"
    )
    parser.add_argument(
        "--ee-site", type=str, default="ee_site",
        help="End-effector site name in XML"
    )
    parser.add_argument(
        "--obj-body", type=str, default="cube",
        help="Object body name in XML"
    )
    parser.add_argument(
        "--table-body", type=str, default="table_top",
        help="Table body name in XML"
    )
    
    args = parser.parse_args()
    
    if not MUJOCO_AVAILABLE:
        logger.error("MuJoCo not available. Install with: pip install mujoco")
        sys.exit(1)
    
    xml_path = Path(args.xml)
    if not xml_path.exists():
        logger.error(f"XML file not found: {xml_path}")
        sys.exit(1)
    
    # Load MuJoCo model
    logger.info(f"Loading model from {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    
    # Create Panda interface
    panda_cfg = MuJoCoPandaConfig(
        ee_site=args.ee_site,
        obj_body=args.obj_body,
        table_body=args.table_body,
    )
    panda = MuJoCoPandaIface(model, data, panda_cfg)
    logger.info(f"Panda interface initialized (EE site: {panda.ee_site_id})")
    
    # Create bandit
    training_cfg = TrainingConfig(
        n_episodes=args.episodes,
        epsilon=args.epsilon,
        learning_rate=args.lr,
        render=args.render,
        render_every=args.render_every,
    )
    vw_cfg = create_vw_config(training_cfg)
    bandit = VWContextualBandit(vw_cfg)
    logger.info(f"VW Bandit initialized (8 actions, 12-dim context)")
    
    # Episode config
    episode_cfg = EpisodeConfig(max_steps=args.max_steps)
    
    # Create runner and train
    runner = PandaBanditRunner(
        model, data, panda, bandit, training_cfg, episode_cfg
    )
    
    results = runner.train()
    
    # Print summary
    print("\n" + "=" * 50)
    print("TRAINING COMPLETE")
    print("=" * 50)
    print(f"Episodes:      {results['total_episodes']}")
    print(f"Total Steps:   {results['total_steps']}")
    print(f"Successes:     {results['successes']} ({results['success_rate']*100:.1f}%)")
    print(f"Collisions:    {results['collisions']}")
    print(f"Avg Reward:    {results['avg_reward']:.3f}")
    print(f"Final 10 Avg:  {results['final_10_avg']:.3f}")
    print("=" * 50)


if __name__ == "__main__":
    main()
