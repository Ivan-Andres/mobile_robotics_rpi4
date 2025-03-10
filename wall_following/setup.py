from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wall_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpi4master',
    maintainer_email='rpi4master@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_alpha =wall_following.control_alpha:main',
            'dist_finder_alpha =wall_following.dist_finder_alpha:main'
        ],
    },
)
