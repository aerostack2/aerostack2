from setuptools import setup

package_name = 'as2_python_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Computer Vision And Aerial Robotics Group (UPM)',
    maintainer_email='cvar.upm3@gmail.com',
    description='Python interface tool',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
    },
)
