from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'urc2027_gazebo'


def model_files():
    data_files = []
    for root, _, files in os.walk('models'):
        if files:
            data_files.append((
                os.path.join('share', package_name, root),
                [os.path.join(root, filename) for filename in files],
            ))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ] + model_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kengo',
    maintainer_email='kengo171227@gmail.com',
    description='Gazebo Sim simulation package for the URC2027 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
