from setuptools import find_packages
from setuptools import setup

setup(
    name='hydrone_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('hydrone_msgs', 'hydrone_msgs.*')),
)
