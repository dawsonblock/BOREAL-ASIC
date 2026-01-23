
import random
import logging
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Any, Dict

logger = logging.getLogger(__name__)

@dataclass
class VWConfig:
    n_actions: int = 8
    context_dim: int = 12
    learning_rate: float = 0.05
    epsilon: float = 0.2
    exploration_strategy: str = "epsilon"
    bits: int = 18
    save_frequency: int = 100
    exploration_budget: float = 1.0
    min_epsilon: float = 0.01

class VWContextualBandit:
    """
    Fallback Linear Epsilon-Greedy Bandit (Pure Python/NumPy).
    
    This implements a simple linear model per action:
        score(a|x) = theta_a . x
    
    It learns via online SGD:
        theta_a = theta_a + lr * (reward - score) * x
        
    This allows the demo to 'learn' without needing the full Vowpal Wabbit C++ library.
    """
    def __init__(self, config: VWConfig):
        self.config = config
        self.n_actions = config.n_actions
        self.dim = config.context_dim
        
        # Initialize weights: one vector per action
        # Small random initialization to break symmetry
        self.weights = np.random.normal(0, 0.01, (self.n_actions, self.dim))
        
        # Bias/Intercept is often useful, but we'll assume features include bias or are sufficient
        # (The context extractor doesn't add bias explicitly, but we can live with it for a demo)
        
        self.epsilon = config.epsilon
        self.lr = config.learning_rate
        self.t = 0
        
        logger.warning(f"USING FALLBACK LINEAR BANDIT (NumPy) - Learning enabled (alpha={self.lr})")
        
    def select_action(self, features: np.ndarray) -> Tuple[int, float]:
        """select_action with epsilon-greedy strategy."""
        # Ensure features are numpy array
        x = np.array(features, dtype=np.float32)
        if x.shape[0] != self.dim:
            # Handle dimension mismatch gracefully if context changes
            if x.shape[0] > self.dim:
                 x = x[:self.dim]
            else:
                 x = np.pad(x, (0, self.dim - x.shape[0]))
        
        # Compute scores for all actions
        scores = self.weights @ x
        
        # Epsilon-greedy
        if random.random() < self.epsilon:
            # Explore
            action = random.randint(0, self.n_actions - 1)
            prob = self.epsilon / self.n_actions + (1 - self.epsilon) if action == np.argmax(scores) else self.epsilon / self.n_actions
        else:
            # Exploit
            action = int(np.argmax(scores))
            prob = 1.0 - self.epsilon + (self.epsilon / self.n_actions)
            
        return action, prob
        
    def update(self, features: Any, action: int, reward: float) -> None:
        """Online SGD update for the selected action's weights."""
        x = np.array(features, dtype=np.float32)
        if x.shape[0] != self.dim:
            if x.shape[0] > self.dim:
                 x = x[:self.dim]
            else:
                 x = np.pad(x, (0, self.dim - x.shape[0]))
                 
        # Prediction
        pred = np.dot(self.weights[action], x)
        
        # Error
        error = reward - pred
        
        # SGD Update: w += lr * error * x
        # Note: We clip error/gradient to prevent explosion in this simple solver
        update = self.lr * error * x
        update = np.clip(update, -1.0, 1.0)
        
        self.weights[action] += update
        self.t += 1
        
        # Decay epsilon slightly
        if self.t % 100 == 0 and self.epsilon > self.config.min_epsilon:
            self.epsilon *= 0.99
            
class RewardNormalizer:
    """Pass-through normalizer (Identity) for simple linear bandit."""
    def __init__(self, alpha=0.01):
        self.alpha = alpha
        
    def normalize(self, reward: float) -> float:
        # For simple SGD, raw rewards often work better than shifting mean to 0
        # especially when penalties are the main signal.
        return reward

class RewardDecayScheduler:
    """Simple discount/decay scheduler."""
    def __init__(self, half_life=10000): 
        pass
    def step(self): 
        pass
    def apply(self, reward): 
        return reward
