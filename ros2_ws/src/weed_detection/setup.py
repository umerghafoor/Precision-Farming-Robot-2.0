from setuptools import setup

package_name = 'weed_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taha',
    maintainer_email='taha@todo.todo',
    description='Weed detection ROS2 package for image processing and AI integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'weed_node = weed_detection.weed_node:main',
    ],
},

)
