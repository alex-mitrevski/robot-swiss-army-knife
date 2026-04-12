import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'vlm_query_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Mitrevski',
    maintainer_email='alemitr@chalmers.se',
    description='A component exposing a VLM query interface',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'query_interface_node = vlm_query_interface.query_interface_node:main'
        ],
    },
)
