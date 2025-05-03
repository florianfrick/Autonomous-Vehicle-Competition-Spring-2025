# setup.py
from setuptools import setup, find_packages

package_name = 'diff_drive_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(), 
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py']),
    ],
    install_requires=['setuptools', 'cvxpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Differential drive MPC simulation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={  
        'console_scripts': [
            'mpc_controller = diff_drive_mpc.mpc_controller:main',
            'simulator = diff_drive_mpc.simulator:main',
        ],
    },
)
