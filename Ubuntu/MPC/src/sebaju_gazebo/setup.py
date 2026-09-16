import os
from glob import glob

from setuptools import setup

package_name = 'sebaju_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alessando Prisco',
    maintainer_email='enzogpt55@libero.it',
    description='SeBaJu balance/jump robot in ROS 2 Humble + Gazebo Fortress',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'wbr_controller = sebaju_gazebo.wbr_controller:main',
            'test_bench_logger = sebaju_gazebo.test_bench_logger:main',
            'wbr_model = sebaju_gazebo.wbr_model:main',
        ],
    },
)
