import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyman',
    maintainer_email='hyman@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sorter_node = arm_control_pkg.sorter_node:main',
            'servo_driver_node = arm_control_pkg.servo_driver_node:main',
            'fake_detector_node = arm_control_pkg.fake_detector_node:main',
        ],
    },
)
