
import sys
import os
import numpy as np

# Add current dir to path
sys.path.append(os.getcwd())

try:
    from rfsn.learning.vw_bandit import VWContextualBandit, VWConfig
    print("Successfully imported VWContextualBandit")
except ImportError as e:
    print(f"Import failed: {e}")
    sys.exit(1)

def test_training():
    print("Testing VW Bandit Training...")
    config = VWConfig(n_actions=3, context_dim=5, quiet=True)
    bandit = VWContextualBandit(config)
    
    # Fake context
    context = np.random.randn(5)
    
    # Select action
    action, _ = bandit.select_action(context)
    print(f"Selected action: {action}")
    
    # Update
    bandit.update(context, action, reward=1.0)
    print("Update successful")
    
    # Save/Load
    import tempfile
    with tempfile.NamedTemporaryFile(suffix='.vw') as f:
        bandit.save(f.name)
        print("Save successful")

if __name__ == "__main__":
    test_training()
