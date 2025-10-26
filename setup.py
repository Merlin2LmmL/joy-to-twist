from setuptools import find_packages, setup

package_name = 'joy_to_twist'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Merlin Ortner',
    maintainer_email='ortnermerlin@gmail.com',
    description='A python script that converts messages from the /joy topic into twist messages for /cmd_vel and Servo commands for the topic /servoX_angle ',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joy_to_twist = joy_to_twist.joy_to_twist:main',
        ],
    },
)
