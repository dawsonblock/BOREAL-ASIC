"""
RFSN v9.2 Main Entry Point
Orchestrates VW bandit for code repair and robotics tasks
"""

import argparse
import logging
import sys
from typing import Optional, Callable, Any, List, Tuple
from pathlib import Path

import numpy as np

from vw_bandit import (
    VWContextualBandit,
    VWConfig,
    VWBanditOptimizer,
    CodeRepairSafetyValidator,
    SafetyValidator,
)
from config import get_config


def setup_logging(log_level: str, log_file: Optional[str] = None):
    """Configure logging"""
    logging.basicConfig(
        level=getattr(logging, log_level),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(log_file) if log_file else logging.NullHandler(),
        ]
    )


class RFSNController:
    """
    Main RFSN controller integrating VW bandit
    
    Supports:
    - Code repair (autonomous bug fixes)
    - Robotics navigation (autonomous control)
    - Custom tasks via context/reward functions
    """
    
    def __init__(self, config_path: Optional[str] = None, env: Optional[str] = None):
        """
        Initialize RFSN controller
        
        Args:
            config_path: Path to YAML config file (optional)
            env: Environment (development, staging, production)
        """
        # Load configuration
        if config_path:
            self.config = self.load_config(config_path)
        else:
            self.config = get_config(env)
        
        # Setup logging
        setup_logging(
            self.config.log_level,
            Path(self.config.log_dir) / "rfsn.log"
        )
        self.logger = logging.getLogger(__name__)
        
        # Initialize VW bandit
        vw_config = VWConfig(**self.config.bandit_config)
        self.bandit = VWContextualBandit(vw_config)
        
        # Initialize optimizer for training
        self.optimizer = VWBanditOptimizer(self.bandit)
        
        self.logger.info(f"RFSN Controller initialized (env: {self.config.environment})")

        # Optional checkpoint manager will be initialised lazily when needed.
        self.checkpoint_manager = None

    
    @staticmethod
    def load_config(path: str):
        """Load configuration from YAML file"""
        from config import RFSNConfig
        return RFSNConfig.from_yaml(path)
    
    def repair_code(
        self,
        error_context: dict,
        repair_strategies: list,
        execute_strategy: Callable
    ) -> tuple:
        """
        Autonomous code repair task
        
        Args:
            error_context: Error information to fix
            repair_strategies: List of available strategies
            execute_strategy: Callable that executes strategy and returns reward
        
        Returns:
            (strategy_name, success, confidence)
        """
        # Extract features from error
        features = self._extract_code_repair_features(error_context)
        
        # Select repair strategy
        action, action_probs = self.bandit.select_action(features)
        strategy = repair_strategies[action]
        
        # Execute and evaluate
        success, reward = execute_strategy(strategy, error_context)
        
        # Update bandit
        self.bandit.update(features, action, reward)
        
        confidence = action_probs[action] if action_probs is not None else None
        
        self.logger.info(
            f"Repair: strategy={strategy}, success={success}, "
            f"confidence={confidence:.3f if confidence else 'N/A'}"
        )
        
        return strategy, success, confidence

    # ------------------------------------------------------------------
    # Optional safety‑aware action selection
    #
    # Many decision tasks have hard constraints that must be enforced before
    # executing an action. This helper wraps bandit action selection with a
    # safety validator. If the initial action is unsafe, it resamples up to
    # `max_resample` times before falling back to the validator's safe default.
    #
    def select_action_with_safety(
        self,
        context: np.ndarray,
        validator: SafetyValidator,
        max_resample: int = 5,
        **context_info: Any,
    ) -> Tuple[int, Optional[np.ndarray]]:
        """
        Select a safe action using the bandit and a SafetyValidator.

        Args:
            context: Feature vector to feed into the bandit
            validator: A SafetyValidator instance to validate actions
            max_resample: Maximum number of times to resample actions if
                unsafe actions are drawn
            context_info: Additional context fields passed to the validator

        Returns:
            (action index, action probabilities or None)
        """
        # Combine context array with any additional fields required for
        # validation into a single dict. The validator may depend on
        # domain‑specific keys like has_backup, target_file, etc.
        safety_context = dict(context_info)
        for attempt in range(max_resample):
            action, probs = self.bandit.select_action(context)
            is_safe, reason = validator.validate(action, safety_context)
            if is_safe:
                return action, probs
            self.logger.debug(
                f"Unsafe action {action} on attempt {attempt + 1}: {reason}. Resampling."
            )
        # If no safe action found, use the validator's default safe action
        default_action = validator.get_safe_default()
        self.logger.error(
            f"Could not find safe action after {max_resample} attempts. "
            f"Returning safe default action {default_action}."
        )
        return default_action, None
    
    def navigate_robot(
        self,
        robot_state: dict,
        action_space: list,
        execute_action: Callable
    ) -> tuple:
        """
        Autonomous robotics navigation task
        
        Args:
            robot_state: Current robot state
            action_space: List of available actions
            execute_action: Callable that executes action and returns new state + reward
        
        Returns:
            (action_name, new_state, reward, confidence)
        """
        # Extract features from robot state
        features = self._extract_robot_features(robot_state)
        
        # Select action
        action_idx, action_probs = self.bandit.select_action(features)
        action = action_space[action_idx]
        
        # Execute and evaluate
        new_state, reward = execute_action(action, robot_state)
        
        # Update bandit
        self.bandit.update(features, action_idx, reward)
        
        confidence = action_probs[action_idx] if action_probs is not None else None
        
        self.logger.debug(
            f"Navigation: action={action}, reward={reward:.3f}, "
            f"confidence={confidence:.3f if confidence else 'N/A'}"
        )
        
        return action, new_state, reward, confidence
    
    @staticmethod
    def _extract_code_repair_features(error_context: dict) -> np.ndarray:
        """
        Extract 64-dimensional features from error context
        
        Features:
        - Error type embedding (0-10)
        - Repository characteristics (11-20)
        - Error frequency/history (21-30)
        - Code structure patterns (31-45)
        - Previous repair attempts (46-55)
        - Temporal information (56-64)
        """
        features = np.zeros(64)
        
        # Error type (normalized hash)
        error_type = error_context.get("type", "unknown")
        features[0] = (hash(error_type) % 10) / 10.0
        
        # Repository size
        repo_size = error_context.get("repo_size", 0)
        features[1] = np.log1p(repo_size) / 20.0
        
        # Error frequency (1-10 scale)
        frequency = min(error_context.get("frequency", 1), 10)
        features[2] = frequency / 10.0
        
        # Complexity (1-10 scale)
        complexity = error_context.get("complexity", 5)
        features[3] = complexity / 10.0
        
        # Previous attempts
        prev_attempts = error_context.get("previous_attempts", 0)
        features[4] = min(prev_attempts, 5) / 5.0
        
        # Recency (hours since first seen)
        recency_hours = error_context.get("recency_hours", 1)
        features[5] = min(np.log1p(recency_hours), 5) / 5.0
        
        # Fill remaining with zeros or additional features as needed
        return features
    
    @staticmethod
    def _extract_robot_features(robot_state: dict) -> np.ndarray:
        """
        Extract 64-dimensional features from robot state
        
        Features:
        - Position (0-2): x, y, z (normalized)
        - Velocity (3-5): vx, vy, vz (normalized)
        - Obstacles (6-13): 8 directions (distance to nearest obstacle)
        - Goal (14-17): relative position, distance, angle
        - Contact (18-21): forces in 4 directions
        - Orientation (22-24): roll, pitch, yaw
        - Joint states (25-40): joint positions (if applicable)
        - Trajectory history (41-55): recent positions
        - Energy (56-60): battery, heat, efficiency
        - Time (61-64): episode progress, time of day, etc.
        """
        features = np.zeros(64)
        
        # Position (normalized to [-1, 1])
        pos = robot_state.get("position", [0, 0, 0])
        features[0:3] = np.tanh(np.array(pos) / 10.0)
        
        # Velocity (normalized)
        vel = robot_state.get("velocity", [0, 0, 0])
        features[3:6] = np.tanh(np.array(vel) / 5.0)
        
        # Obstacle distances (8 directions, normalized)
        obstacles = robot_state.get("obstacle_distances", [10.0] * 8)
        features[6:14] = np.tanh(np.array(obstacles) / 10.0)
        
        # Goal information
        goal_rel = robot_state.get("goal_relative", [0, 0])
        goal_dist = robot_state.get("goal_distance", 10.0)
        features[14:17] = np.tanh([goal_rel[0] / 10.0, goal_rel[1] / 10.0, goal_dist / 20.0])
        
        # Contact forces
        contacts = robot_state.get("contact_forces", [0.0] * 4)
        features[18:22] = np.tanh(np.array(contacts) / 100.0)
        
        # Orientation (roll, pitch, yaw normalized to [-1, 1])
        euler = robot_state.get("euler_angles", [0, 0, 0])
        features[22:25] = np.tanh(np.array(euler) / 3.14159)
        
        # Joint states (if applicable)
        joints = robot_state.get("joint_angles", [0] * 16)
        features[25:41] = np.tanh(np.array(joints[:16]) / 3.14159)
        
        # Trajectory history (recent positions)
        traj = robot_state.get("trajectory", [[0, 0]] * 8)
        for i, pos_i in enumerate(traj[:8]):
            features[41 + 2*i: 41 + 2*i + 2] = np.tanh(np.array(pos_i) / 10.0)
        
        # Energy
        battery = robot_state.get("battery", 100.0) / 100.0
        features[57] = np.clip(battery, 0, 1)
        
        # Time (episode progress)
        episode_progress = robot_state.get("episode_progress", 0.0)
        features[58] = np.clip(episode_progress, 0, 1)
        
        return features
    
    def train(
        self,
        context_generator: Callable,
        n_episodes: int = 100,
        episode_length: int = 100,
        epsilon_decay: float = 0.01,
        checkpoint_freq: int = 10
    ) -> dict:
        """
        Train bandit over multiple episodes
        
        Args:
            context_generator: Callable returning (context, reward_fn) tuples
            n_episodes: Number of episodes
            episode_length: Steps per episode
            epsilon_decay: Linear epsilon decay rate
            checkpoint_freq: Save every N episodes
        
        Returns:
            Training history dictionary
        """
        history = self.optimizer.train(
            context_generator,
            n_episodes=n_episodes,
            episode_length=episode_length,
            epsilon_decay=epsilon_decay,
            checkpoint_freq=checkpoint_freq
        )
        
        self.logger.info(f"Training completed: {n_episodes} episodes")
        return history
    
    def save_model(self, path: Optional[str] = None) -> str:
        """
        Save current bandit model
        
        Args:
            path: Optional path. If None, uses timestamped path in model_dir
        
        Returns:
            Path where model was saved
        """
        if path is None:
            from datetime import datetime
            path = str(
                Path(self.config.model_dir) /
                f"bandit_{datetime.now().strftime('%Y%m%d_%H%M%S')}.vw"
            )
        
        self.bandit.save(path)
        self.logger.info(f"Model saved to {path}")
        return path
    
    def load_model(self, path: str) -> None:
        """
        Load bandit model from disk
        
        Args:
            path: Path to saved model
        """
        self.bandit.load(path)
        self.logger.info(f"Model loaded from {path}")


# -----------------------------------------------------------------------------
# Shadow evaluation infrastructure
#
# The ShadowEvaluator runs the learned policy in parallel with a baseline
# policy (e.g. random or heuristic) on the same contexts. It collects
# comparative statistics to determine whether the bandit policy consistently
# outperforms the baseline. This is useful for validating improvements prior
# to full deployment.


class ShadowEvaluator:
    def __init__(self, bandit: VWContextualBandit, baseline_policy: Callable[[np.ndarray], int], name: str = "shadow_eval") -> None:
        self.bandit = bandit
        self.baseline = baseline_policy
        self.name = name
        self.comparison_history: List[dict] = []
        self.n_comparisons = 0

    def evaluate_single(self, context: np.ndarray, reward_fn: Callable[[int], float]) -> dict:
        """
        Compare bandit and baseline on a single context.
        Returns a dict containing actions, rewards and win indicator.
        """
        bandit_action, _ = self.bandit.select_action(context)
        baseline_action = self.baseline(context)
        bandit_reward = float(reward_fn(bandit_action))
        baseline_reward = float(reward_fn(baseline_action))
        bandit_won = bandit_reward > baseline_reward
        result = {
            "bandit_action": int(bandit_action),
            "baseline_action": int(baseline_action),
            "bandit_reward": bandit_reward,
            "baseline_reward": baseline_reward,
            "bandit_won": bandit_won,
        }
        self.comparison_history.append(result)
        self.n_comparisons += 1
        return result

    def evaluate_batch(self, contexts: np.ndarray, reward_fn: Callable[[int], float], n_trials: int = 1000) -> dict:
        """
        Evaluate the bandit and baseline on a batch of contexts. The contexts
        array should be of shape (n_trials, context_dim). Returns summary
        statistics including win ratio and average rewards.
        """
        wins = 0
        bandit_total = 0.0
        baseline_total = 0.0
        # Determine the number of trials to run
        trials = min(n_trials, len(contexts))
        for i in range(trials):
            context = contexts[i]
            result = self.evaluate_single(context, reward_fn)
            if result["bandit_won"]:
                wins += 1
            bandit_total += result["bandit_reward"]
            baseline_total += result["baseline_reward"]
        # Summary statistics
        win_ratio = wins / trials if trials > 0 else 0.0
        bandit_avg = bandit_total / trials if trials > 0 else 0.0
        baseline_avg = baseline_total / trials if trials > 0 else 0.0
        stats = {
            "trials": trials,
            "bandit_avg_reward": bandit_avg,
            "baseline_avg_reward": baseline_avg,
            "bandit_wins": wins,
            "win_ratio": win_ratio,
            "bandit_better": (win_ratio > 0.5 and (bandit_avg - baseline_avg) > 0.01),
        }
        return stats


def main():
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description="RFSN v9.2 - Autonomous Code Repair & Robotics with VW Bandit"
    )
    
    parser.add_argument(
        "--env",
        choices=["development", "staging", "production"],
        default="development",
        help="Environment configuration"
    )
    parser.add_argument(
        "--config",
        type=str,
        help="Path to custom YAML configuration"
    )
    parser.add_argument(
        "--task",
        choices=["repair", "robotics", "train"],
        default="train",
        help="Task to run"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=100,
        help="Number of training episodes"
    )
    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Logging level"
    )
    parser.add_argument(
        "--model",
        type=str,
        help="Path to load existing model"
    )
    
    args = parser.parse_args()
    
    # Initialize controller
    controller = RFSNController(config_path=args.config, env=args.env)
    
    # Load model if specified
    if args.model:
        controller.load_model(args.model)
    
    # Run task
    if args.task == "train":
        print("Training mode not yet implemented. Use library API directly.")
    elif args.task == "repair":
        print("Code repair mode not yet implemented. Use library API directly.")
    elif args.task == "robotics":
        print("Robotics mode not yet implemented. Use library API directly.")
    
    print(f"RFSN v9.2 - {args.env} environment")


if __name__ == "__main__":
    main()
