from setuptools import setup
import os
from glob import glob

package_name = 'ardupilot_supervisory'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Supervisory system for ArduPilot fixed-wing aircraft',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'power_monitor = ardupilot_supervisory.power_monitor:main',
            'fault_detector = ardupilot_supervisory.fault_detector:main',
            'mission_supervisor = ardupilot_supervisory.mission_supervisor:main',
            'energy_manager = ardupilot_supervisory.energy_manager:main',
            'telemetry_handler = ardupilot_supervisory.telemetry_handler:main',
            'emergency_manager = ardupilot_supervisory.emergency_manager:main',
        ],
    },
)
