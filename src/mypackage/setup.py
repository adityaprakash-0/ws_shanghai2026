import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mypackage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vmx',
    maintainer_email='adityaprakash.main@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_allterrain = mypackage.task_manager_allterrain:main',
            'task_manager_grip = mypackage.task_manager_grip:main',
            'custom_teleop_allterrain = mypackage.custom_teleop_allterrain:main',
            'hold_to_move_teleop = mypackage.hold_to_move_teleop:main',
            'task_manager_mecanum = mypackage.task_manager_mecanum:main',
            'task_manager_mecanum_colore = mypackage.task_manager_mecanum_colore:main',
            'colore_detect = mypackage.colore_detect_node:main',
            'colore_calibrate = mypackage.colore_calibrate_node:main',
            'task_manager_xdrive = mypackage.task_manager_xdrive:main',
            'light_tower_play = mypackage.light_tower_play:main',
            'merged_scan_clearance = mypackage.merged_scan_clearance:main',
            'oms_node = mypackage.oms_node:main',
            'oms_joy_node = mypackage.oms_joy_node:main',
        ],
    },
)
