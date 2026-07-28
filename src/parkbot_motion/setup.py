from glob import glob

from setuptools import find_packages, setup

package_name = 'parkbot_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='tmdwodl12@gmail.com',
    description=(
        '주행 제어·기구학 순수 로직 라이브러리 (body_twist_toward, 메카넘 기구학, 휠오도, '
        '축/뎁스 감지) + pose_controller_node(R3, NavigateToPose) + '
        'axle_detector_node(R4, 측면 뎁스->축중심) + lift_action_server(R4, ControlLift) + '
        'ingress_node(R5a, 차 밑 진입: 뎁스 중앙유지 + 축 중간값 정지) + '
        'pickup_orchestrator_node(R5b, ExecuteParkingTask: 2로봇 픽업 안무)'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_controller_node = parkbot_motion.pose_controller_node:main',
            'axle_detector_node = parkbot_motion.axle_detector_node:main',
            'lift_action_server = parkbot_motion.lift_action_server:main',
            'ingress_node = parkbot_motion.ingress_node:main',
            'pickup_orchestrator_node = parkbot_motion.pickup_orchestrator_node:main',
            'carry_action_server = parkbot_motion.carry_action_server:main',
            'exit_carry_action_server = parkbot_motion.exit_carry_action_server:main',
            'exit_pickup_orchestrator_node = parkbot_motion.exit_pickup_orchestrator_node:main',
            'user_request_gateway_node = parkbot_motion.user_request_gateway_node:main',
        ],
    },
)
