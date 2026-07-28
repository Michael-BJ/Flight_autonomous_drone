from glob import glob

from setuptools import find_packages, setup

package_name = 'fm_deploy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/mavros_only.launch.py',
            'launch/fm_real.launch.py',
            'launch/fm_all.launch.py',
        ]),
        # Checkpoint FM yang ikut ter-install (opsional — boleh kosong).
        ('share/' + package_name + '/model/fm', glob('model/fm/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michaelbryanjahanto@gmail.com',
    description='FM-Planner deployment on real hardware (Jetson Orin Nano + PX4 + Gemini 2)',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'px4_sensor_reader          = fm_deploy.px4_sensor_reader:main',
            'mavros_tf_broadcaster_node = fm_deploy.mavros_tf_broadcaster_node:main',
            'gemini2_depth_bridge_node  = fm_deploy.gemini2_depth_bridge_node:main',
            'depth_to_pointcloud_node   = fm_deploy.depth_to_pointcloud_node:main',
            'fm_inference_real_node     = fm_deploy.fm_inference_real_node:main',
        ],
    },
)
