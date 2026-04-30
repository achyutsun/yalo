from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yalo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', ['rviz/exploration.rviz']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOLO Team',
    maintainer_email='achyutros@gmail.com',
    description='YALO Mobile Robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yalo = yalo.yalo:main',
            'decision_maker = yalo.decision_maker:main',
            'frontier_detector = yalo.frontier_detector:main',
            'entropy_explorer = yalo.entropy_explorer:main',
        ],
    },
)
