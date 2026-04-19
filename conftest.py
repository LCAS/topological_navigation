"""
Pytest configuration for topological_navigation workspace.

This file ensures that the topological_navigation package can be imported
during testing by adding the source and build directories to the Python path.
"""

import sys
from pathlib import Path

# The ROS package directory that contains the actual Python package
_pkg_dir = Path(__file__).parent / 'topological_navigation'
if _pkg_dir.exists() and str(_pkg_dir) not in sys.path:
    sys.path.insert(0, str(_pkg_dir))

# Also try the build directory (for colcon builds)
_build_dir = Path(__file__).parent / 'build' / 'topological_navigation'
if _build_dir.exists() and str(_build_dir) not in sys.path:
    sys.path.insert(0, str(_build_dir))


def pytest_configure(config):
    """Pytest hook that runs before test collection.

    Ensures the source and build directories are in the Python path.
    """
    for d in (_pkg_dir, _build_dir):
        if d.exists() and str(d) not in sys.path:
            sys.path.insert(0, str(d))
