from setuptools import find_packages, setup

package_name = 'vision_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'canny_node = vision_pkg.canny_node:main',
  'bright_spot_tracker = vision_pkg.bright_spot_tracker:main',
  'bright_spot_follower = vision_pkg.bright_spot_follower:main',
        ],
    },
)
