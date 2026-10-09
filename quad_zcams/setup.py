from glob import glob

from setuptools import find_packages, setup

package_name = 'quad_zcams'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models/amanita_river_v4_yolo11n_openvino_model',
         glob('models/amanita_river_v4_yolo11n_openvino_model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lachlan Dimovski',
    maintainer_email='lachlan.dimovski02@gmail.com',
    description='Parrot with a down and an up depth camera: height above ground and safe climbs over canopies',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zcam_quad_node = quad_zcams.zcam_quad_node:main',
            'rover_capture_node = quad_zcams.rover_capture_node:main',
            'ps4_teleop = quad_zcams.ps4_teleop:main',
            'fleet_quad_node = quad_zcams.fleet_quad_node:main',
            'fleet_coordinator = quad_zcams.fleet_coordinator:main',
            'fleet_yolo_pool = quad_zcams.fleet_yolo_pool:main',
        ],
    },
)
