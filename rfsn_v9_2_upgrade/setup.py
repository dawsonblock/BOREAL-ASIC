"""
RFSN (Reactive Framework for Semantic Navigation)
Complete Build Configuration with VW Integration
"""

from setuptools import setup, find_packages
import os

# Read long description from README
with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="rfsn",
    version="9.2.0",
    author="RFSN Team",
    author_email="team@rfsn.dev",
    description="Reactive Framework for Semantic Navigation - AI-powered code repair and autonomous robotics",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/rfsn/rfsn",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
    python_requires=">=3.9",
    install_requires=[
        "numpy>=1.21.0",
        "scipy>=1.7.0",
        "torch>=2.0.0",
        "transformers>=4.30.0",
        "vowpalwabbit>=9.6.0",  # VW Core - 600x faster than NumPy
        "pydantic>=2.0.0",
        "python-dotenv>=0.19.0",
        "requests>=2.28.0",
        "matplotlib>=3.5.0",
        "pandas>=1.3.0",
        "scikit-learn>=1.1.0",
        "tqdm>=4.62.0",
        "pyyaml>=6.0",
        "dataclasses-json>=0.5.7",
        "jsonschema>=4.0.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=3.0.0",
            "pytest-asyncio>=0.20.0",
            "black>=22.0.0",
            "isort>=5.10.0",
            "flake8>=4.0.0",
            "mypy>=0.950",
            "sphinx>=4.5.0",
            "sphinx-rtd-theme>=1.0.0",
        ],
        "robotics": [
            "pybullet>=3.1.0",
            "gym>=0.21.0",
            "dm-control>=1.0.0",
        ],
        "benchmarks": [
            "pytest-benchmark>=3.4.0",
            "memory-profiler>=0.60.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "rfsn=rfsn.cli:main",
        ],
    },
    include_package_data=True,
    zip_safe=False,
    project_urls={
        "Bug Tracker": "https://github.com/rfsn/rfsn/issues",
        "Documentation": "https://rfsn.readthedocs.io",
        "Source Code": "https://github.com/rfsn/rfsn",
    },
)
