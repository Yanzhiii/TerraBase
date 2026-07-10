from glob import glob
from setuptools import setup

package_name = 'lite3_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TerraBase',
    maintainer_email='user@example.com',
    description='Lite3 topic bridge for running AEDE in a Docker container.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lite3_bridge_node = lite3_bridge.bridge_node:main',
        ],
    },
)
