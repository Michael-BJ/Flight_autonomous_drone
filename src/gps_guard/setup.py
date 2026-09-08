from setuptools import find_packages, setup

package_name = 'gps_guard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/gps_guard.launch.py',
            'launch/gps_guard_all.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michaelbryanjahanto@gmail.com',
    description='Standalone GPS pre-flight / stability checker for PX4 via MAVROS.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gps_check_node = gps_guard.gps_check_node:main',
        ],
    },
)
