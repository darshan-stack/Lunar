from setuptools import setup
import os
from glob import glob

package_name = 'amfs_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darshan',
    maintainer_email='darshan@example.com',
    description='Adaptive Multi-Sensor Fusion SLAM for lunar environments',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adaptive_slam_node = amfs_slam.adaptive_slam_node:main',
            'simple_adaptive_slam_node = amfs_slam.simple_adaptive_slam:main',
        ],
    },
)

