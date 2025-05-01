from setuptools import setup

package_name = 'black_line_detector_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sheetal',
    maintainer_email='sheetal@example.com',
    description='Detects black lines using OpenCV from camera feed',
    license='MIT',
    entry_points={
        'console_scripts': [
            'black_line_detector = black_line_detector_pkg.black_line_node:main',
        ],
    },
)
