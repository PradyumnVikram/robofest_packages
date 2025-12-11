from setuptools import find_packages
from setuptools import setup

setup(
    name='swarm_control',
    version='0.0.0',
    packages=find_packages(
        include=('swarm_control', 'swarm_control.*')),
)
