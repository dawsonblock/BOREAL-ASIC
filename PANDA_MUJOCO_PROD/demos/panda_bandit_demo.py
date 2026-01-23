#!/usr/bin/env python3
"""
Entry point for running the Panda bandit training demo.

This script simply forwards execution to the ``main`` function from
``rfsn.panda_bandit.run_mujoco_panda_bandit``.  Exposing a thin wrapper
here provides a stable command for users to invoke the bandit training
without having to know the internal module layout.

Example usage::

    python3 demos/panda_bandit_demo.py --xml assets/franka_panda_safe.xml --episodes 10 --render

For more options, run::

    python3 demos/panda_bandit_demo.py --help

"""
import os
import sys

# Add project root to path so 'rfsn' module can be found
current_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.dirname(current_dir)
if root_dir not in sys.path:
    sys.path.append(root_dir)

from rfsn.panda_bandit.run_mujoco_panda_bandit import main


if __name__ == "__main__":
    # Delegate to the underlying main() to parse arguments and run training
    main()
