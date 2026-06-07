from setuptools import setup
import os
from glob import glob

package_name = 'go2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
        (os.path.join('share', package_name, 'python_site'),
         glob('python_site/*.py')),
        (os.path.join('share', package_name, 'compat', 'aioice'),
         glob('compat/aioice/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TerraBase',
    maintainer_email='user@example.com',
    description='Go2-AEDE bridge nodes and keyboard teleop',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_sdk_driver_node = go2_bridge.go2_sdk_driver_wrapper:main',
            'go2_bridge_node = go2_bridge.bridge_node:main',
            'joint_state_republisher = go2_bridge.joint_state_republisher:main',
            'keyboard_teleop = go2_bridge.keyboard_teleop:main',
            'stepp_depth_projector = go2_bridge.stepp_depth_projector:main',
        ],
    },
)
