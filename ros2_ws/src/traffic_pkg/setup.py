from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'traffic_pkg'

model_dir = os.path.join('models', 'best_openvino_model')
model_files = [p for p in glob(os.path.join(model_dir, '*')) if os.path.isfile(p)]


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
              'crosswalk = traffic_pkg.crosswalk:main',
              'crosswalk_slow = traffic_pkg.crosswalk_slow:main',
              'crosswalk_slow_stop_cmd = traffic_pkg.crosswalk_slow_stop_cmd:main',
              'crosswalk_tpu = traffic_pkg.crosswalk_tpu:main',
              'crosswalk_slow_stop_cmd_imu = traffic_pkg.crosswalk_slow_stop_cmd_imu:main',
        ],
    },
)
