from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'oms'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ADEV',
    maintainer_email='adityaprakash.main@gmail.com',
    description='OMS: rotation, lift, linear, servos, boot homing',
    license='MIT',
    # Direct scripts (not console_scripts): importlib.metadata cannot resolve
    # colcon symlink egg-link installs under sudo.
    scripts=[
        'scripts/oms_all',
        'scripts/oms_test',
    ],
)
