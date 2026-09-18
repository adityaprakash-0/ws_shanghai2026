from setuptools import find_packages, setup

package_name = 'skill23_lookup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lookup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ADEV',
    maintainer_email='adityaprakash.main@gmail.com',
    description='OCR / ArUco waypoint lookup for Skill23 Studio',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lookup_node = skill23_lookup.lookup_node:main',
            'prefetch_models = skill23_lookup.prefetch_models:main',
        ],
    },
)
