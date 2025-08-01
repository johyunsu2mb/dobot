from setuptools import setup, find_packages

setup(
    name="dobot-control-system",
    version="1.0.0",
    description="Improved Dobot Control System with Python TCP Server and ROS2 Client",
    author="Your Name",
    author_email="your.email@example.com",
    packages=find_packages(),
    install_requires=[
        "flask>=2.3.0",
        "requests>=2.31.0",
        "pyyaml>=6.0.0",
        "coloredlogs>=15.0.0",
        "python-dotenv>=1.0.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.4.0",
            "pytest-mock>=3.11.0",
            "black>=23.7.0",
            "flake8>=6.0.0",
        ]
    },
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "dobot-server=python_server.main:main",
            "dobot-gui=python_server.dobot_gui:main",
        ]
    },
)