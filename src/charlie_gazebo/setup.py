from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'charlie_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*.*')),
        (os.path.join('share', package_name,'config'), glob('config/*.*')),
        (os.path.join('share', package_name,'rviz_config'), glob('rviz_config/*.*')),
        (os.path.join('share', package_name,'models'), glob('models/*.*')),
        (os.path.join('share', package_name,'plot_juggler'), glob('plot_juggler/*.*')),
        (os.path.join('share', package_name,'maps'), glob('maps/*.*')),
        (os.path.join('share', package_name,'slam_data'), glob('slam_data/*.*')),
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
        ],
    },
)
