from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skill23_mission'


def _map_data_files():
    """Install maps/<name>/* preserving subfolders."""
    entries = []
    maps_root = 'maps'
    if not os.path.isdir(maps_root):
        return entries
    for name in sorted(os.listdir(maps_root)):
        d = os.path.join(maps_root, name)
        if not os.path.isdir(d):
            continue
        files = [os.path.join(d, f) for f in os.listdir(d)
                 if os.path.isfile(os.path.join(d, f))]
        if files:
            entries.append(
                (os.path.join('share', package_name, 'maps', name), files))
    return entries


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            [f for f in glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config', 'nav2_params'),
            glob('config/nav2_params/*')),
        (os.path.join('share', package_name, 'config', 'hw_params'),
            glob('config/hw_params/*')),
        *_map_data_files(),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='ADEV',
    maintainer_email='adityaprakash.main@gmail.com',
    description='Standalone Nav2 mission runner',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_runner = skill23_mission.mission_runner:main',
        ],
    },
)
