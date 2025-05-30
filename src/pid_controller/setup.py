from setuptools import find_packages, setup

package_name = 'pid_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtlecar',
    maintainer_email='turtlecar@todo.todo',
    description='Simple PID controller that will subscribe to sensor topics and then publish Twist messages to the cmd_vel topic.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_node = pid_controller.pid_node:main',
            'pid_school = pid_controller.pid_school:main',
            'pid_stopsign = pid_controller.pid_stopsign:main',
        ],
    }
)
