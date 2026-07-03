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
            'custom_teleop_allterrain = mypackage.custom_teleop_allterrain:main',
            'task_manager_mecanum = mypackage.task_manager_mecanum:main',
        ],
    },
)
