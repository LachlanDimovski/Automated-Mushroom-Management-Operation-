from glob import glob

from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # Trained YOLO weights, resolved from the install share dir so the
        # node needs no per-machine path.
        ('share/' + package_name + '/models', glob('models/*.pt')),
        # Interchangeable variants, swappable at runtime via the model_path
        # parameter. See models/variants/README.md.
        ('share/' + package_name + '/models/variants',
            glob('models/variants/*.pt') + glob('models/variants/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lachlan Dimovski',
    maintainer_email='lachlan.dimovski02@gmail.com',
    description='Shared perception (YOLO11 detector) for AMMO robots',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = perception.yolo_node:main',
        ],
    },
)
