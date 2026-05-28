import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'inspection_eoat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='macs',
    maintainer_email='macs@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stepper_light_controller_node = inspection_eoat.stepper_light_controller_node:main',
            'imu_processor_node = inspection_eoat.imu_processor_node:main',
        ],
    },
)
