from setuptools import setup
import os
from glob import glob

package_name = 'weed_detection_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taha Sajid Awan',
    maintainer_email='taha@example.com',
    description='ROS2 node for ArUco marker detection and image processing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_processor = weed_detection_node.aruco_processor:main',
            'image_publisher = weed_detection_node.image_publisher:main',
            'image_viewer = weed_detection_node.image_viewer:main',
            'pretty_viewer = weed_detection_node.pretty_viewer:main',
        ],
    },
)

