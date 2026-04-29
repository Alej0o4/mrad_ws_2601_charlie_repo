from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'charlie_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
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
            'open_loop_odom_node = charlie_bringup.open_loop_odom:main',
            'bicycle_odom_node = charlie_odom.bicycle_odom_node:main',
            'rf2o_odom_node = charlie_odom.RF2O_odometry:main',
        ],
    },
)
