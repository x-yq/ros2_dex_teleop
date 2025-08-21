from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_driver'

setup(
    name=package_name,  # This should match your package name exactly
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yeqiao.xu@campus.tu-berlin.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver_node = robot_driver.robot_driver_node:main'
        ],
    },
)