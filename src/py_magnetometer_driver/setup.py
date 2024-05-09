from setuptools import find_packages, setup

package_name = 'py_magnetometer_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/' + package_name + '/launch', ['launch/magnetometer_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhody',
    maintainer_email='rhodyboland@hotmail.com',
    description='ROS 2 magnetometer driver',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'magnetometer_node = py_magnetometer_driver.magnetometer_node:main'
        ],
    },
)
