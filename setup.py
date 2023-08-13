from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odrive_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='Zsolt Egri',
    maintainer_email='i@zsoltegri.com',
    description='A ROS2 driver for ODrive',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_node = odrive_ros2.odrive_node:main'
        ],
    },

)
