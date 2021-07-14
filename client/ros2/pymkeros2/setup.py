from setuptools import setup
import os
from glob import glob

package_name = 'pymkeros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*_config.yaml')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='magikeye',
    maintainer_email='jigar@magik-eye.com',
    description='MKEROS2 Python package for publishing 3D point cloud data provided by Magik Eye sensors.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pymkeros2_node = pymkeros2.pymkeros2_node:main',
            'device_info = pymkeros2.device_info:main',
        ],
    },
    include_package_data=True,
)
