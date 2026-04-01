# storage: setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'omx_inference'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'omx_storage_ai_node = omx_inference.omx_storage_ai:main',
        ],
    },
)