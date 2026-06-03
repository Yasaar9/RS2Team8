from setuptools import setup
from glob import glob
import os

package_name = 'rs2_team8'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}/nodes', f'{package_name}/utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch',         glob('launch/*.py')),
        (f'share/{package_name}/config/maps',    glob('config/maps/*')),
        (f'share/{package_name}/config/params',  glob('config/params/*')),
        (f'share/{package_name}/config/rviz',    glob('config/rviz/*')),
        (f'share/{package_name}/urdf',           glob('urdf/*')),
        (f'share/{package_name}/worlds',         glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RS2 Team 8',
    maintainer_email='jerry.z.sun@student.uts.edu.au',
    description='RS2 Team 8 — Social Tour Guide Robot Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = rs2_team8.nodes.navigation:main',
            'send_waypoint   = rs2_team8.nodes.send_waypoint:main',
            'ui_node         = rs2_team8.nodes.ui:main',
        ],
    },
)