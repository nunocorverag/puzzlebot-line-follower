from glob import glob

from setuptools import find_packages, setup

package_name = 'puzzlebot_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puzzlebot',
    maintainer_email='luis.montellano@outlook.com',
    description='Puzzlebot line follower and traffic-light ROS2 nodes.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Active line-follower / traffic-light stack.
            'line_follower = puzzlebot_ros.line_follower:main',
            'traffic_light = puzzlebot_ros.traffic_light:main',
            'pictures = puzzlebot_ros.pictures:main',          # camera-intrinsics capture
            'stopnoise = puzzlebot_ros.stopnoise:main',        # emergency stop helper
            # Nodes from earlier course modules (square/waypoint/kalman/MPC/aruco)
            # were moved to archive/ and intentionally are not exposed here.
        ],
    },
)
