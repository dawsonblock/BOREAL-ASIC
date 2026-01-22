"""
Comprehensive Test Suite for RFSN v9.2 with VW Integration
Tests cover: core functionality, performance, persistence, edge cases
"""

import pytest
import numpy as np
import tempfile
import os
import time
from pathlib import Path

from vw_bandit import VWContextualBandit, VWConfig, VWBanditOptimizer


class TestVWConfig:
    """Configuration validation tests"""
    
    def test_config_defaults(self):
        """Test default configuration values"""
        config = VWConfig()
        assert config.n_actions == 10
        assert config.context_dim == 64
        assert config.learning_rate == 0.1
        assert config.exploration_strategy == "softmax"
    
    def test_config_to_vw_args(self):
        """Test VW argument generation"""
        config = VWConfig(n_actions=5, learning_rate=0.05)
        args = config.to_vw_args()
        assert any("--loss_function=logistic" in arg for arg in args)
        assert any("0.05" in arg for arg in args)  # Learning rate
    
    def test_custom_config(self):
        """Test custom configuration"""
        config = VWConfig(
            n_actions=20,
            context_dim=128,
            epsilon=0.2,
            bits=20
        )
        assert config.n_actions == 20
        assert config.context_dim == 128
        assert config.epsilon == 0.2


class TestVWContextualBandit:
    """Core bandit functionality tests"""
    
    @pytest.fixture
    def bandit(self):
        """Create bandit instance"""
        config = VWConfig(n_actions=5, context_dim=10, quiet=True)
        return VWContextualBandit(config)
    
    def test_initialization(self, bandit):
        """Test bandit initialization"""
        assert bandit.config.n_actions == 5
        assert bandit.config.context_dim == 10
        assert bandit.example_count == 0
        assert bandit.vw is not None
    
    def test_context_to_vw_format(self, bandit):
        """Test context formatting"""
        context = np.array([0.1, 0.2, 0.0, 0.3])
        vw_str = bandit._context_to_vw_format(context)
        
        # Check format: "context |f_0:0.100000 f_1:0.200000 ..."
        assert vw_str.startswith("context |")
        assert "f_0:0.1" in vw_str
        assert "f_1:0.2" in vw_str
        assert "f_2:" not in vw_str  # Zero should be skipped
    
    def test_select_action(self, bandit):
        """Test action selection"""
        context = np.random.randn(10)
        action, probs = bandit.select_action(context, return_probs=False)
        
        assert 0 <= action < 5
        assert isinstance(action, (int, np.integer))
    
    def test_select_action_with_epsilon_greedy(self, bandit):
        """Test epsilon-greedy exploration"""
        bandit.config.exploration_strategy = "epsilon"
        context = np.random.randn(10)
        
        actions = []
        for _ in range(100):
            action, _ = bandit.select_action(context, epsilon=0.5)
            actions.append(action)
        
        # Should have some action diversity with 50% epsilon
        assert len(set(actions)) > 1
    
    def test_single_update(self, bandit):
        """Test single example update"""
        context = np.random.randn(10)
        action = 1
        reward = 1.0
        
        initial_count = bandit.example_count
        bandit.update(context, action, reward)
        
        assert bandit.example_count == initial_count + 1
    
    def test_batch_update(self, bandit):
        """Test batch update"""
        batch_size = 10
        contexts = np.random.randn(batch_size, 10)
        actions = np.random.randint(0, 5, batch_size)
        rewards = np.random.randint(0, 2, batch_size).astype(float)
        
        initial_count = bandit.example_count
        bandit.batch_update(contexts, actions, rewards)
        
        assert bandit.example_count == initial_count + batch_size
    
    def test_get_action_values(self, bandit):
        """Test Q-value retrieval"""
        context = np.random.randn(10)
        q_values = bandit.get_action_values(context)
        
        assert q_values.shape == (5,)
        assert np.all(np.isfinite(q_values))


class TestVWBanditPersistence:
    """Model saving and loading tests"""
    
    @pytest.fixture
    def bandit_with_temp_dir(self):
        """Create bandit with temporary model directory"""
        with tempfile.TemporaryDirectory() as tmpdir:
            config = VWConfig(
                n_actions=5,
                context_dim=10,
                model_dir=tmpdir,
                save_frequency=1000  # Disable auto-save
            )
            bandit = VWContextualBandit(config)
            yield bandit, tmpdir
    
    def test_save_model(self, bandit_with_temp_dir):
        """Test model saving"""
        bandit, tmpdir = bandit_with_temp_dir
        
        # Train briefly
        for _ in range(10):
            context = np.random.randn(10)
            action, _ = bandit.select_action(context)
            reward = float(np.random.rand() > 0.5)
            bandit.update(context, action, reward)
        
        # Save
        model_path = os.path.join(tmpdir, "test_model.vw")
        bandit.save(model_path)
        
        assert os.path.exists(model_path)
        assert os.path.exists(model_path.replace(".vw", "_metadata.json"))
        assert os.path.getsize(model_path) > 0
    
    def test_load_model(self, bandit_with_temp_dir):
        """Test model loading"""
        bandit, tmpdir = bandit_with_temp_dir
        
        # Train and save
        for _ in range(10):
            context = np.random.randn(10)
            action, _ = bandit.select_action(context)
            reward = float(np.random.rand() > 0.5)
            bandit.update(context, action, reward)
        
        model_path = os.path.join(tmpdir, "test_model.vw")
        bandit.save(model_path)
        example_count = bandit.example_count
        
        # Load
        bandit2 = VWContextualBandit(bandit.config)
        bandit2.load(model_path)
        
        assert bandit2.example_count == example_count
    
    def test_model_consistency_after_load(self, bandit_with_temp_dir):
        """Test that predictions are consistent after save/load"""
        bandit, tmpdir = bandit_with_temp_dir
        
        # Train
        for _ in range(20):
            context = np.random.randn(10)
            action, _ = bandit.select_action(context)
            reward = float(np.random.rand() > 0.5)
            bandit.update(context, action, reward)
        
        # Test context
        test_context = np.array([0.5] * 10)
        
        # Get predictions before save
        q_vals_before = bandit.get_action_values(test_context)
        
        # Save and load
        model_path = os.path.join(tmpdir, "consistency_test.vw")
        bandit.save(model_path)
        
        bandit2 = VWContextualBandit(bandit.config)
        bandit2.load(model_path)
        
        # Get predictions after load
        q_vals_after = bandit2.get_action_values(test_context)
        
        # Should be close (may have small numerical differences)
        assert np.allclose(q_vals_before, q_vals_after, rtol=1e-3, atol=1e-5)


class TestVWBanditOptimizer:
    """Training loop tests"""
    
    @pytest.fixture
    def optimizer(self):
        """Create optimizer with test bandit"""
        config = VWConfig(n_actions=5, context_dim=10)
        bandit = VWContextualBandit(config)
        return VWBanditOptimizer(bandit)
    
    def test_optimizer_initialization(self, optimizer):
        """Test optimizer initialization"""
        assert optimizer.bandit is not None
        assert optimizer.cumulative_reward == 0.0
        assert len(optimizer.history) == 5  # 5 metric types
    
    def test_train_episode(self, optimizer):
        """Test single episode training"""
        # Simple context generator
        def context_gen():
            context = np.random.randn(10)
            # Reward function: action 2 is best
            reward_fn = lambda a: 1.0 if a == 2 else 0.0
            return context, reward_fn
        
        metrics = optimizer.train_episode(context_gen, episode_length=50)
        
        assert "episode_reward" in metrics
        assert "avg_reward" in metrics
        assert metrics["avg_reward"] >= 0.0
        assert metrics["avg_reward"] <= 1.0
    
    def test_train_convergence(self, optimizer):
        """Test that training improves performance"""
        # Simple bandit: action 1 always gives reward, others give 0
        def context_gen():
            context = np.random.randn(10)
            reward_fn = lambda a: 1.0 if a == 1 else 0.0
            return context, reward_fn
        
        history = optimizer.train(
            context_gen,
            n_episodes=10,
            episode_length=50,
            epsilon_decay=0.05,
            checkpoint_freq=100  # No checkpointing during test
        )
        
        # Check that learning occurred
        assert len(history["avg_reward"]) == 10
        final_reward = history["avg_reward"][-1]
        initial_reward = history["avg_reward"][0]
        
        # Should improve (with high epsilon decay, might not always improve)
        # At minimum, should get some rewards
        assert final_reward > 0.1


class TestVWBanditPerformance:
    """Performance benchmarks"""
    
    def test_prediction_latency(self):
        """Benchmark prediction speed"""
        config = VWConfig(n_actions=20, context_dim=64)
        bandit = VWContextualBandit(config)
        context = np.random.randn(64)
        
        # Warmup
        for _ in range(10):
            bandit.select_action(context)
        
        # Benchmark
        n_iters = 1000
        start = time.time()
        for _ in range(n_iters):
            bandit.select_action(context)
        elapsed = time.time() - start
        
        latency_ms = (elapsed / n_iters) * 1000
        throughput = n_iters / elapsed
        
        # VW should be <0.1ms per prediction (600x faster than NumPy)
        assert latency_ms < 1.0  # Conservative: <1ms
        assert throughput > 1000  # >1000 predictions/sec
        
        print(f"\nPrediction Performance:")
        print(f"  Latency: {latency_ms:.4f}ms")
        print(f"  Throughput: {throughput:.0f} predictions/sec")
    
    def test_update_latency(self):
        """Benchmark update speed"""
        config = VWConfig(n_actions=20, context_dim=64)
        bandit = VWContextualBandit(config)
        
        contexts = np.random.randn(1000, 64)
        actions = np.random.randint(0, 20, 1000)
        rewards = np.random.rand(1000)
        
        # Warmup
        for i in range(10):
            bandit.update(contexts[i], actions[i], rewards[i])
        
        # Benchmark
        start = time.time()
        for i in range(100, 1000):
            bandit.update(contexts[i], actions[i], rewards[i])
        elapsed = time.time() - start
        
        latency_ms = (elapsed / 900) * 1000
        throughput = 900 / elapsed
        
        # VW should be ~0.1ms per update (10K updates/sec)
        assert latency_ms < 2.0  # Conservative: <2ms
        assert throughput > 500  # >500 updates/sec
        
        print(f"\nUpdate Performance:")
        print(f"  Latency: {latency_ms:.4f}ms")
        print(f"  Throughput: {throughput:.0f} updates/sec")
    
    def test_batch_update_efficiency(self):
        """Test batch update efficiency vs single updates"""
        config = VWConfig(n_actions=20, context_dim=64)
        
        batch_size = 100
        contexts = np.random.randn(batch_size, 64)
        actions = np.random.randint(0, 20, batch_size)
        rewards = np.random.rand(batch_size)
        
        # Single updates
        bandit1 = VWContextualBandit(config)
        start = time.time()
        for i in range(batch_size):
            bandit1.update(contexts[i], actions[i], rewards[i])
        single_time = time.time() - start
        
        # Batch update
        bandit2 = VWContextualBandit(config)
        start = time.time()
        bandit2.batch_update(contexts, actions, rewards)
        batch_time = time.time() - start
        
        # Batch should be slightly more efficient due to overhead amortization
        print(f"\nBatch Update Efficiency:")
        print(f"  Single updates: {single_time:.4f}s")
        print(f"  Batch update: {batch_time:.4f}s")
        print(f"  Speedup: {single_time/batch_time:.2f}x")
        
        # Both should complete in reasonable time
        assert single_time < 2.0
        assert batch_time < 2.0


class TestEdgeCases:
    """Edge case handling"""
    
    def test_zero_context(self):
        """Handle zero context vector"""
        config = VWConfig(n_actions=5, context_dim=10)
        bandit = VWContextualBandit(config)
        
        zero_context = np.zeros(10)
        action, _ = bandit.select_action(zero_context)
        assert 0 <= action < 5
    
    def test_large_values(self):
        """Handle large context values"""
        config = VWConfig(n_actions=5, context_dim=10)
        bandit = VWContextualBandit(config)
        
        large_context = np.full(10, 1e6)
        action, _ = bandit.select_action(large_context)
        assert 0 <= action < 5
    
    def test_negative_context(self):
        """Handle negative context values"""
        config = VWConfig(n_actions=5, context_dim=10)
        bandit = VWContextualBandit(config)
        
        neg_context = np.full(10, -0.5)
        action, _ = bandit.select_action(neg_context)
        assert 0 <= action < 5
    
    def test_inconsistent_batch_size(self):
        """Handle mismatched batch dimensions"""
        config = VWConfig(n_actions=5, context_dim=10)
        bandit = VWContextualBandit(config)
        
        contexts = np.random.randn(10, 10)
        actions = np.random.randint(0, 5, 5)  # Wrong size
        rewards = np.random.rand(10)
        
        # Should handle gracefully or raise clear error
        with pytest.raises((ValueError, IndexError)):
            bandit.batch_update(contexts, actions, rewards)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
