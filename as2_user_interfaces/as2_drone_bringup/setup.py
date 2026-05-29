import os
from glob import glob
from setuptools import setup

package_name = 'as2_drone_bringup'

setup(
    name=package_name,
    version='1.1.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'scripts'),
         glob(os.path.join('scripts', '*.sh'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CVAR-UPM',
    maintainer_email='cvar.upm3@gmail.com',
    description='AeroStack2 drone bringup',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={},
)
