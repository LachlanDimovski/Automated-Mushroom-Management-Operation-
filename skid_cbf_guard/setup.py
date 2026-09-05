from glob import glob

from setuptools import find_packages, setup

package_name = 'skid_cbf_guard'

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
    description='Depth-based control barrier function safety filter (geometric barrier, closed-form filter)',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cbf_guard_node = skid_cbf_guard.cbf_guard_node:main',
        ],
    },
)
