"""Import the package from this source tree, not a stale install."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))


def pytest_configure(config):
    config.addinivalue_line('markers', 'slow: full missions in the kinematic sim')
