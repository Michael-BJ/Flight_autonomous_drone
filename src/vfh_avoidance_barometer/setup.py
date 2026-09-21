from setuptools import find_packages, setup

package_name = 'vfh_avoidance_barometer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/vfh_all_barometer.launch.py',
            'launch/vfh_perception.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michaelbryanjahanto@gmail.com',
    description='VFH (Vector Field Histogram) obstacle avoidance on the Orbbec Gemini 2 depth camera, '
                'flown on forward_move with barometric altitude hold (z from the barometer, x/y from GPS/EKF).',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'vfh_avoidance_node   = vfh_avoidance_barometer.vfh_avoidance_node:main',
            'vfh_flight_baro_node = vfh_avoidance_barometer.vfh_flight_baro_node:main',
        ],
    },
)
