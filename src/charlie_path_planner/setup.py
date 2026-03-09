from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'charlie_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejo',
    maintainer_email='ajejo.lizcano@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dijkstra_node = charlie_path_planner.charlie_path_planner_dijkstra:main',
            'dijkstra_node_v3 = charlie_path_planner.charlie_path_planner_dijkstra_v3:main',
            'ara_star_node = charlie_path_planner.charlie_path_planner_ARA:main',
            'waypoint_manager_node = charlie_path_planner.waypoint_manager:main',
        ],
    },
)
