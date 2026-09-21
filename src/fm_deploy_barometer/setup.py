from setuptools import find_packages, setup

package_name = 'fm_deploy_barometer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/fm_real_barometer.launch.py',
            'launch/fm_all_barometer.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michaelbryanjahanto@gmail.com',
    description='fm_deploy with barometric altitude hold and return-along-the-flown-path recovery. Planner and model unchanged.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'fm_inference_baro_node     = fm_deploy_barometer.fm_inference_baro_node:main',
            'fm_inference_recovery_node = fm_deploy_barometer.fm_inference_recovery_node:main',
            # NEW (2026-09-21, MAPVIEW): read-only terminal map viewer.
            'octomap_view_node          = fm_deploy_barometer.octomap_view_node:main',
        ],
    },
)
