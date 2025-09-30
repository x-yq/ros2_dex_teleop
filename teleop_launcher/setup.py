from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'teleop_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'launch']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yeqiao.xu@campus.tu-berlin.de',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
