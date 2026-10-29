from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'synora_robot_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kunal',
    maintainer_email='user@todo.todo',
    description='4-DOF Robotic Arm with two-finger gripper simulation in Gazebo Harmonic for ROS 2 Jazzy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = synora_robot_arm.arm_controller:main',
            'gripper_controller = synora_robot_arm.gripper_controller:main',
            'joint_state_monitor = synora_robot_arm.joint_state_monitor:main',
        ],
    },
)

