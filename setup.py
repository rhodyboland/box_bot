from setuptools import setup

package_name = 'box_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/box_bot.launch.py']),
        # Add other launch files if necessary
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package for ROS 2 master launch files.',
    license='Apache 2.0 or appropriate license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # If you had any executable scripts, they would be listed here.
        ],
    },
)
