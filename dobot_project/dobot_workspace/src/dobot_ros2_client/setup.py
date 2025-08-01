from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dobot_ros2_client'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # 패키지 디렉토리 명시
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 client for Dobot robot control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dobot_client_node = dobot_ros2_client.dobot_client_node:main',
            'dobot_test_node = dobot_ros2_client.dobot_test_node:main',
        ],
    },
)
