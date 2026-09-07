from glob import glob

from setuptools import find_packages, setup

package_name = 'ammo_gui_node'

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
    # PySide6 is a pip dependency, not an apt/rosdep one - install it once
    # per machine with: pip3 install PySide6 (see README.md in this package).
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Danil Vasiliev',
    maintainer_email='danil.vasiliev@gmail.com',
    description='Interactive PySide6 operator GUI: mission setup, live progress, '
                 'mushroom count, pause/resume, manual Parrot teleop, and an '
                 'occupancy-grid map view - separate from ammo_dashboard.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = ammo_gui_node.main:main',
        ],
    },
)
