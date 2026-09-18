from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skill23_studio'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), [
            f for f in glob('config/*')
            if os.path.isfile(f)
        ]),
        (os.path.join('share', package_name, 'config', 'align'),
            glob('config/align/*')),
        (os.path.join('share', package_name, 'config', 'cobra'),
            glob('config/cobra/*')),
        (os.path.join('share', package_name, 'config', 'nav2_params'),
            glob('config/nav2_params/*')),
        (os.path.join('share', package_name, 'config', 'hw_params'),
            glob('config/hw_params/*')),
        (os.path.join('share', package_name, 'models', 'yolov8n_ncnn'),
            glob('models/yolov8n_ncnn/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ADEV',
    maintainer_email='adityaprakash.main@gmail.com',
    description='WorldSkills Studio backend',
    license='MIT',
    entry_points={
        'console_scripts': [
            'studio_bridge = skill23_studio.studio_bridge_node:main',
            'studio_align = skill23_studio.nav.align_yaw_node:main',
            'studio_align_test = skill23_studio.nav.align_test_trigger_node:main',
            'studio_cobra = skill23_studio.nav.cobra_align_node:main',
            'studio_cobra_test = skill23_studio.nav.cobra_test_trigger_node:main',
            'studio_detect_align = skill23_studio.nav.detect_align_node:main',
            'studio_detect_align_test = skill23_studio.nav.detect_align_test_node:main',
            'studio_detect_align_oms_test = skill23_studio.nav.detect_align_oms_test_node:main',
            'studio_encoder_distance_template = skill23_studio.nav.encoder_distance_template_node:main',
            'studio_stop_stack = skill23_studio.process_manager:main_stop',
        ],
    },
)
