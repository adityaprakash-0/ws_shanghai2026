from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'adev_dashboard'


def _launch_py_files():
    files = []
    for path in glob('launch/*.py'):
        if os.path.isfile(path) and not os.path.islink(path):
            files.append(path)
        elif os.path.islink(path) and os.path.exists(path):
            files.append(path)
    return files


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), _launch_py_files()),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'scripts', 'mode_nav2'), glob('scripts/mode_nav2/*.sh')),
        (os.path.join('share', package_name, 'scripts', 'mode_nav2_guard'), glob('scripts/mode_nav2_guard/*.sh')),
        (os.path.join('share', package_name, 'scripts', 'mode_nav2_secret_mode'), glob('scripts/mode_nav2_secret_mode/*.sh')),
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
