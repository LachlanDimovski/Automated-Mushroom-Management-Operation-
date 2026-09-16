from glob import glob

from setuptools import find_packages, setup

package_name = 'ammo_mission_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lachlan Dimovski',
    maintainer_email='lachlan.dimovski02@gmail.com',
    description="Cross-platform mission coordination between the skidsteer and quadcopter",
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quad_follow_node = ammo_mission_control.follow_node:main',
            'mushroom_mission_node = ammo_mission_control.mushroom_mission_node:main',
            'quad_search_node = ammo_mission_control.quad_search_node:main',
            'cage_capture_node = ammo_mission_control.cage_capture_node:main',
            'truth_localizer = ammo_mission_control.truth_localizer:main',
        ],
    },
)
