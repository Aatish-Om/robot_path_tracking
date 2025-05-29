from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_path_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.path_tracker'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Turtlebot3 path tracking assignment package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main_tracker = turtlebot3_path_tracking.main_tracker:main',
        ],
    },
)
