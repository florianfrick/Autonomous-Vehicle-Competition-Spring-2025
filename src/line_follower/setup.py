from setuptools import setup

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sheetal',
    maintainer_email='sheetal@example.com',
    description='Line following robot using Canny edge detection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'line_follower_node = line_follower.line_follower_node:main',
            'black_line_detector = line_follower.black_line_detector:main',
        ],
    },
)
