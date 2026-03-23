from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'charlie_path_tracking'

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
            'pure_pursuit_v1 = charlie_path_tracking.pure_pursuit_v1:main',
            'mpc_controller_node = charlie_path_tracking.mpc_controller_node:main',
            'stanley_node = charlie_path_tracking.stanley_node:main',
        ],
    },
)
