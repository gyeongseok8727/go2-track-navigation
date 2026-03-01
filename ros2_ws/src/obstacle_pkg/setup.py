from setuptools import find_packages, setup
from glob import glob

package_name = 'obstacle_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seok',
    maintainer_email='gyeongseok8727@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'obstacle_v1 = obstacle_pkg.avoid_go_v1:main',
            'obstacle_v2 = obstacle_pkg.avoid_go_v2:main',
            'obstacle_slow = obstacle_pkg.avoid_go_slow:main',
            'obstacle_oneshot = obstacle_pkg.avoid_go_oneshot_move:main',
        ],
    },
)
