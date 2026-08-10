import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autonomous_wheelchair'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'udev'),
            glob(os.path.join('udev', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inu',
    maintainer_email='user@example.com',
    description='Autonomous wheelchair navigation stack (base driver, safety monitor, waypoint navigation)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver = autonomous_wheelchair.base_driver:main',
            'safety_monitor = autonomous_wheelchair.safety_monitor:main',
            'waypoint_navigator = autonomous_wheelchair.waypoint_navigator:main',
            'wasd_teleop = autonomous_wheelchair.wasd_teleop:main',
        ],
    },
)
