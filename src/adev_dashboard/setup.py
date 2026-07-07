from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'adev_dashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ADEV',
    maintainer_email='adityaprakash.main@gmail.com',
    description='ADEV Dashboard lightweight bridge for WorldSkills navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'adev_bridge = adev_dashboard.bridge_node:main',
            'adev_simulator = adev_dashboard.simulator_node:main',
        ],
    },
)
