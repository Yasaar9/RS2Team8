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
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/config/maps', glob('config/maps/*')),
        (f'share/{package_name}/config/rviz', glob('config/rviz/*')),
        (f'share/{package_name}/urdf', glob('urdf/*')),
        (f'share/{package_name}/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JerryZsun',
    maintainer_email='jerry.z.sun@student.uts.edu.au',
    description='Social Tour Guide Robot Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = rs2_team8.nodes.navigation:main',
            'send_waypoint = rs2_team8.nodes.send_waypoint:main',
            'ui_node = rs2_team8.nodes.ui:main',
        ],
    },
)

# NOTE: waypoints.ini lives alongside navigation.py in the source tree at
#   rs2_team8/nodes/waypoints.ini
# The navigation node resolves the path relative to its own __file__ at
# runtime, so it reads the source file directly whether you use
# --symlink-install or not. No install-time copying is needed.
# When deploying to the real robot, copy the whole rs2_team8/ folder as-is
# and the ini file travels with the code.