from glob import glob

from setuptools import find_packages, setup

package_name = 'parkbot_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 측위 지도. 이 패키지가 소유하며 런타임에 노드가 읽는다.
        ('share/' + package_name + '/data', glob('data/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='tmdwodl12@gmail.com',
    description='ArUco 바닥 마커 검출·자세추정 노드 (Phase 1 측위, M2)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_detector = parkbot_aruco.aruco_detector:main',
            'marker_localizer_node = parkbot_aruco.marker_localizer_node:main',
        ],
    },
)
