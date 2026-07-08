from setuptools import find_packages, setup

package_name = 'takeoff_land'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/mavros_only.launch.py',
            'launch/takeoff_land.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michaelbryanjahanto@gmail.com',
    description='Minimal OFFBOARD takeoff & landing for PX4 via MAVROS.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'px4_sensor_reader   = takeoff_land.px4_sensor_reader:main',
            'takeoff_land_node   = takeoff_land.takeoff_land_node:main',
            'mode_monitor        = takeoff_land.mode_monitor:main',
            'hold_position_node  = takeoff_land.hold_position_node:main',
        ],
    },
)
