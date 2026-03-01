from setuptools import find_packages, setup
import os

package_name = 'video_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'unitree-sdk2py'],
    zip_safe=True,
    maintainer='seok',
    maintainer_email='gyeongseok8727@gmail.com',
    description="ROS 2 nodes: frame publisher for Unitree Go2 and OpenCV viewer.",
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
        	'frame_publisher_node = video_pkg.frame_publisher_node:main',
        	'viewer_node = video_pkg.viewer_node:main',

        ],
    },
)
