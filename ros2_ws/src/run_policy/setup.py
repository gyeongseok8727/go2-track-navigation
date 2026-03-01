from setuptools import setup

package_name = 'run_policy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='ROS2: /cmd_vel -> UDP bridge and demo publisher.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'udp_bridge_node = run_policy.udp_bridge_node:main',
            'demo_cmdvel_pub = run_policy.demo_cmdvel_pub:main',
        ],
    },
)
