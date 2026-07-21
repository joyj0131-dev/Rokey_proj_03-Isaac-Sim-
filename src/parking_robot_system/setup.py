from setuptools import find_packages, setup

package_name = 'parking_robot_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/parking_robot_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youngjin',
    maintainer_email='joyj01312@gmail.com',
    description='주차 로봇 시스템 노드 뼈대 패키지',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_request_gateway = parking_robot_system.user_request_gateway:main',
            'task_dispatcher = parking_robot_system.task_dispatcher:main',
            'parking_slot_manager = parking_robot_system.parking_slot_manager:main',
            'robot_task_orchestrator = parking_robot_system.robot_task_orchestrator:main',
            'safety_monitor = parking_robot_system.safety_monitor:main',
            'vehicle_detection_node = parking_robot_system.vehicle_detection_node:main',
            'navigate_action_server = parking_robot_system.navigate_action_server:main',
            'align_action_server = parking_robot_system.align_action_server:main',
            'lift_action_server = parking_robot_system.lift_action_server:main',
        ],
    },
)
