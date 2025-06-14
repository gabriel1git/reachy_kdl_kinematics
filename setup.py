import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'reachy_kdl_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pollen Robotics',
    maintainer_email='contact@pollen-robotics.com',
    description='ROS2 Humble package for Reachy 2023 kinematics (URDF, arms and head FK/IK). ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ikine = reachy_kdl_kinematics.reachy_kdl_kinematics_node:main',
            'pub_ikine = reachy_kdl_kinematics.inverse_kinematics:main',
            'publisher_move = reachy_kdl_kinematics.pub_move:main'
        ],
    },
)
