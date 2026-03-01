from setuptools import find_packages, setup

package_name = 'python_sdk'

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
    maintainer='mr',
    maintainer_email='mr@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'sport_client = python_sdk.go2_sport_client:main',
            'go2_pub = python_sdk.go2_pub:main',
        ],
    },
)
