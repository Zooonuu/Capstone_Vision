import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capstone_vision',
    maintainer_email='staroseeun@gmail.com',
    description='YOLO11 detection, ground-plane projection (IPM) and map-frame '
                'object tracking for the camera line of the pipeline.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_image_publisher_node = robot_perception.webcam_image_publisher_node:main',
            'yolo_detector_node = robot_perception.yolo_detector_node:main',
        ],
    },
)
