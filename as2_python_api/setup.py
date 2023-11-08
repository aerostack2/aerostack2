from setuptools import setup

package_name = 'as2_python_api'

setup(
    name=package_name,
    version='1.0.5',
    packages=[package_name],
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
