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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lachlan Dimovski',
    maintainer_email='lachlan.dimovski02@gmail.com',
    description='YOLOv8 object detection on the Husky skid-steer camera feed',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = skidsteer_perception.yolo_detector:main',
        ],
    },
)
