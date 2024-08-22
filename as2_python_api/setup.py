from setuptools import setup, find_packages

package_name = 'as2_python_api'

setup(
    name=package_name,
    version='1.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/hooks', ['hooks/resource_paths.sh']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CVAR-UPM',
    maintainer_email='cvar.upm3@gmail.com',
    description='Python interface tool',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_executor = as2_python_api.mission_interpreter.ros2_adapter:main',
        ],
    },
)
