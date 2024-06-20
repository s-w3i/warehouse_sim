from setuptools import find_packages, setup

package_name = 'robot_control'

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
    maintainer='usern',
    maintainer_email='w3i.0425@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to_waypoint = robot_control.move_to_waypoint:main',
            'fake_action_server = robot_control.fake_action_server:main',
            'fake_action_client = robot_control.fake_action_client:main',
            'battery_simulator = robot_control.battery_simulator:main',
            'state_publisher = robot_control.state_publisher:main',
        ],
    },
)
