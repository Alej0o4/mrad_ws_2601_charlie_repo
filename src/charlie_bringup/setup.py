from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'charlie_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.*')),
        (os.path.join('share', package_name,'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejo',
    maintainer_email='alejandro.marin92@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_recorder = charlie_bringup.path_recorder:main',
            'vicon_to_odom_node = charlie_bringup.pose2odom_node:main',
            'scan_inverter_node = charlie_bringup.lidar_node:main',
            'open_loop_odom_node = charlie_bringup.open_loop_odom:main',
            'startup_node = charlie_bringup.startup_node:main',
            'imu_processor_node = charlie_bringup.imu_node:main',
        ],
    },
)
