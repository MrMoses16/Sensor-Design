import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lidar_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Added this line so ROS 2 copies your launch files during the build
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kristopher T',
    maintainer_email='02kmtellez@gmail.com',
    description='Mini fence obstacle avoidance for RPLidar and Cube Black',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mini_fence_node = lidar_driver.mini_fence_node:main'
        ],
    },
)