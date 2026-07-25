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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='tmdwodl12@gmail.com',
    description='주행 제어·기구학 순수 로직 라이브러리 (body_twist_toward, 메카넘 기구학, 휠오도, 축/뎁스 감지) — 아직 노드 없음, R1 이관',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
