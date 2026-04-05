import os
from setuptools import setup
from glob import glob

package_name = 'semantic_segmentation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Mitrevski',
    maintainer_email='alemitr@chalmers.se',
    description='A package with utilities for semantic segmentation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segment_objects = semantic_segmentation.segment_objects:main'
        ],
    },
)
