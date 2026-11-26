from glob import glob

from setuptools import find_packages, setup

package_name = 'ff_magpie_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antlered-Snek',
    maintainer_email='devinhew01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ethernet_reader = ff_magpie_ros.ethernet_reader:main',
            'ld06_dtof_lidar_parser = ff_magpie_ros.ld06_dtof_lidar_parser:main',
            'data_preplanner = ff_magpie_ros.data_preplanner:main',
        ],
    },
)
