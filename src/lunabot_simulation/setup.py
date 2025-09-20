from setuptools import setup
import os
from glob import glob

package_name = 'lunabot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Add world files  
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        # Add URDF files
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darshan',
    maintainer_email='darshan@example.com',
    description='LunaBot simulation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

