from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'rob499_rover_status_ui'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.py'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.xml'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Wyatt Boer',
    maintainer_email='wyattboer@gmail.com',
    description='ROS2 implementation for improved rover status ui',
    license='BSD-3-Clause',

    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #test listening node for topics/nodes, needs to be confirmed working
            'testlisten = rob499_rover_status_ui.testlisten:main',
       ],
    },
)
