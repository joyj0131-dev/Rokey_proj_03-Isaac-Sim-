from setuptools import find_packages
from setuptools import setup

setup(
    name='parking_robot_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('parking_robot_interfaces', 'parking_robot_interfaces.*')),
)
