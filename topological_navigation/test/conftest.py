"""
Pytest configuration for topological_navigation tests.

This file ensures that the topological_navigation package can be imported
during testing by adding the source directory to the Python path and
invalidating any stale module cache.
"""

import sys
from pathlib import Path

# The ROS package directory containing the actual Python package.
# Layout: topological_navigation/ (ROS pkg) -> topological_navigation/ (python pkg)
_pkg_dir = str(Path(__file__).resolve().parent.parent)

# Insert BEFORE anything else to win over the outer __init__.py
if _pkg_dir not in sys.path:
    sys.path.insert(0, _pkg_dir)

# Invalidate the root cached module so Python re-discovers from the correct path.
# Only remove the root module – submodules will be imported fresh.
# We must not remove test.conftest itself.
_stale = 'topological_navigation'
if _stale in sys.modules:
    _cached_file = getattr(sys.modules[_stale], '__file__', '') or ''
    # Only invalidate if the cached module points to the wrong location
    if 'topological_navigation/topological_navigation' not in _cached_file:
        del sys.modules[_stale]
