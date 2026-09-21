from setuptools import find_packages, setup

package_name = 'takeoff_land_barometer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/takeoff_land_barometer.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michaelbryanjahanto@gmail.com',
    description='takeoff_land with barometric altitude hold (z from the barometer, x/y from GPS/EKF).',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'takeoff_land_baro_node = takeoff_land_barometer.takeoff_land_baro_node:main',
        ],
    },
)
