import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'skill23_test'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'reports'), glob('reports/.gitkeep')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ADEV',
    maintainer_email='adityaprakash.main@gmail.com',
    description='Wheel odometry calibration tests',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_calibrator = skill23_test.odom_calibrator:main',
            'imu_calibrator = skill23_test.imu_calibrator:main',
            'ekf_covariance = skill23_test.ekf_covariance:main',
        ],
    },
)
