from setuptools import find_packages
from setuptools import setup

setup(
    name='biguasim_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('biguasim_interfaces', 'biguasim_interfaces.*')),
)
