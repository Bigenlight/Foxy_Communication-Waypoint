from setuptools import setup
import os
from glob import glob

package_name = 'communication_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='Launch file for communication packages',
    license='License declaration',
    entry_points={
        'console_scripts': [],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
)
