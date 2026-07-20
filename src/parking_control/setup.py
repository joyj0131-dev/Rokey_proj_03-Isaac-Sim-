from glob import glob

from setuptools import find_packages, setup

package_name = 'parking_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/db', glob('db/*.sql')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youngjin',
    maintainer_email='joyj01312@gmail.com',
    description='Team A 관제 계층: task_dispatcher, parking_slot_manager와 순수 Python 코어',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_slot_manager = parking_control.parking_slot_manager_node:main',
            'task_dispatcher = parking_control.task_dispatcher_node:main',
            'sim_orchestrator = parking_control.sim_orchestrator_node:main',
            'safety_monitor = parking_control.safety_monitor_node:main',
        ],
    },
)
