from glob import glob

from setuptools import find_packages, setup

package_name = 'skidsteer_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Trained YOLO weights, so the node resolves them from the install
        # share dir rather than a hard-coded per-machine path.
        ('share/' + package_name + '/models', glob('models/*.pt')),
        # Interchangeable variants (real-photo vs sim-trained). Installed so
        # they can be swapped at runtime via the node's model_path parameter
        # without editing or rebuilding anything.
        ('share/' + package_name + '/models/variants',
            glob('models/variants/*.pt') + glob('models/variants/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lachlan Dimovski',
    maintainer_email='lachlan.dimovski02@gmail.com',
    description='YOLO11 mushroom detection on the Husky skid-steer camera feed',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = skidsteer_perception.yolo_detector:main',
            'capture_sim_dataset = skidsteer_perception.capture_sim_dataset:main',
        ],
    },
)
