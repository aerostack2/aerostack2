import os
from glob import glob
from setuptools import setup

package_name = 'as2_keyboard_teleoperation'

setup(
    name=package_name,
    version='1.1.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*_launch.py'))),
        ('share/' + package_name,
         [package_name + '/keyboard_teleoperation.py']),
        ('share/' + package_name,
         [package_name + '/drone_manager.py']),
        ('share/' + package_name,
         [package_name + '/main_window.py']),
        ('share/' + package_name,
         [package_name + '/settings_window.py']),
        ('share/' + package_name,
         [package_name + '/localization_window.py']),
        ('share/' + package_name,
         [package_name + '/config_values.py']),
        ('share/' + package_name + '/config',
         ['config/teleop_values_config.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CVAR-UPM',
    maintainer_email='cvar.upm3@gmail.com',
    description='Python interface tool',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={},
)
