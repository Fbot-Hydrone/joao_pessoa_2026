"""Import the packages from this source tree (not a stale install)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
# the general confined-navigation package lives next door
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'hydrone_confined'))


def pytest_configure(config):
    config.addinivalue_line('markers', 'slow: full mission scenarios in the headless sim')
