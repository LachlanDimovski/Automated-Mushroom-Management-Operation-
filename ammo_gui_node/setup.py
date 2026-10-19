import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ammo_gui_node'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test', 'tests', 'tests.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'layouts'), glob('layouts/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Danil',
    maintainer_email='danil.vasiliev@gmail.com',
    description='AMMO operator GUI and Debug UI (PySide6).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gui = ammo_gui_node.main:main',
            'debug = ammo_gui_node.debug_main:main',
            'mission_bridge = ammo_gui_node.mission_bridge:main',
        ],
    },
)
