"""
RFSN v9.2 Core Configuration
Unified settings for VW bandit, logging, and deployment
"""

import os
from dataclasses import dataclass
from typing import Optional
from pathlib import Path

import yaml


@dataclass
class RFSNConfig:
    """Master RFSN Configuration"""
    
    # Environment
    environment: str = "development"  # development, staging, production
    debug: bool = True
    log_level: str = "INFO"
    
    # VW Bandit Configuration
    bandit_config: dict = None
    
    # Storage
    model_dir: str = "./models"
    log_dir: str = "./logs"
    data_dir: str = "./data"
    
    # Server (if running inference server)
    server_host: str = "0.0.0.0"
    server_port: int = 8000
    server_workers: int = 4
    
    # Monitoring
    metrics_enabled: bool = True
    metrics_interval: int = 60  # seconds
    
    # Development
    profile_enabled: bool = False
    profile_output: str = "./profiles"
    
    @classmethod
    def from_yaml(cls, path: str) -> "RFSNConfig":
        """Load configuration from YAML file"""
        with open(path, "r") as f:
            config_dict = yaml.safe_load(f)
        return cls(**config_dict)
    
    def save_yaml(self, path: str) -> None:
        """Save configuration to YAML file"""
        with open(path, "w") as f:
            yaml.dump(self.__dict__, f)
    
    def create_directories(self) -> None:
        """Create required directories"""
        for directory in [self.model_dir, self.log_dir, self.data_dir]:
            Path(directory).mkdir(parents=True, exist_ok=True)


# Default configurations for different environments
DEFAULT_CONFIG = {
    "development": {
        "environment": "development",
        "debug": True,
        "log_level": "DEBUG",
        "bandit_config": {
            "n_actions": 10,
            "context_dim": 64,
            "learning_rate": 0.1,
            "epsilon": 0.2,  # Higher exploration in dev
            "bits": 18,
            "save_frequency": 10,
        },
        "profile_enabled": True,
    },
    "staging": {
        "environment": "staging",
        "debug": False,
        "log_level": "INFO",
        "bandit_config": {
            "n_actions": 10,
            "context_dim": 64,
            "learning_rate": 0.08,
            "epsilon": 0.1,
            "bits": 19,
            "save_frequency": 100,
        },
        "profile_enabled": False,
    },
    "production": {
        "environment": "production",
        "debug": False,
        "log_level": "WARNING",
        "bandit_config": {
            "n_actions": 10,
            "context_dim": 64,
            "learning_rate": 0.05,
            "epsilon": 0.05,  # Lower exploration in prod
            "bits": 20,
            "save_frequency": 1000,
        },
        "metrics_enabled": True,
        "metrics_interval": 30,
        "profile_enabled": False,
    },
}


def get_config(env: Optional[str] = None) -> RFSNConfig:
    """
    Get configuration for environment
    
    Args:
        env: Environment name (development, staging, production)
             If None, uses RFSN_ENV environment variable or defaults to development
    """
    if env is None:
        env = os.getenv("RFSN_ENV", "development")
    
    if env not in DEFAULT_CONFIG:
        raise ValueError(f"Unknown environment: {env}")
    
    config_dict = DEFAULT_CONFIG[env]
    config = RFSNConfig(**config_dict)
    config.create_directories()
    return config
