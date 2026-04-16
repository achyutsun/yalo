from setuptools import find_packages, setup

package_name = 'yalo'

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
            'frontier_detector = yalo.frontier_detector:main',
            'cluster_goal_selector = yalo.cluster_goal_selector:main',
        ],
    },
)
