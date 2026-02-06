from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'charlie_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'sensors'), glob('sensors/*.*')),
        (os.path.join('share', package_name,'charlie_robot_urdf'), glob('charlie_robot_urdf/*.*')),
        (os.path.join('share', package_name,'macros'), glob('macros/*.*')),
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
