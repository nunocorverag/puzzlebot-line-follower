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
            'line_detector = puzzlebot_ros.line_detector:main',
            'line_follower = puzzlebot_ros.line_follower:main',
            'mpc_node = puzzlebot_ros.mpc_node:main',
            'odom_node = puzzlebot_ros.odom_node:main',
            'pictures = puzzlebot_ros.pictures:main',
            'pid_square_controller = puzzlebot_ros.pid_square_controller:main',
            'pid_waypoint_follower = puzzlebot_ros.pid_waypoint_follower:main',
            'stopnoise = puzzlebot_ros.stopnoise:main',
            'traffic_light = puzzlebot_ros.traffic_light:main',
            'trafficlight_waypoint = puzzlebot_ros.trafficlight_waypoint:main',
            'trajectory_generator = puzzlebot_ros.trajectory_generator:main',
            'vision_node = puzzlebot_ros.vision_node:main',
        ],
    },
)
