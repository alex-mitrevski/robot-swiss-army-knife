import os
from setuptools import setup
from glob import glob

package_name = 'object_pose_estimator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ninad Dixit',
    maintainer_email='ninad.h.dixit@gmail.com',
    description='Kalman-filtered object pose estimation with occlusion-aware state machine',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_estimator = object_pose_estimator.pose_estimator_node:main',
        ],
    },
)
