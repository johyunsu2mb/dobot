from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dobot_robot_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일들 (존재하는 경우에만)
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py') if os.path.exists('launch') else []),
        # Config 파일들 (존재하는 경우에만)  
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml') if os.path.exists('config') else []),
        # Scripts 파일들 (존재하는 경우에만)
        (os.path.join('share', package_name, 'scripts'), 
         glob('scripts/*.py') if os.path.exists('scripts') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johyunsu2mb',
    maintainer_email='johyunsu61@gmail.com',
    description='Multi-robot management system for Dobot robots with TCP and ROS2 communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dobot_server = dobot_robot_manager.server_node:main',
            'dobot_client = dobot_robot_manager.client_node:main',
            'dobot_tcp_server = dobot_robot_manager.tcp_server:main',
        ],
    },
)