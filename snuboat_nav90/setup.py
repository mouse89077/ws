import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'snuboat_nav90'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaboat',
    maintainer_email='j6847110@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar = snuboat_nav90.lidar_converter:main',
            'obs_odom = snuboat_nav90.obstacle_avoidance_odom:main',
            'obs_slam = snuboat_nav90.obstacle_avoidance_slam:main',
            'obs_slam3d = snuboat_nav90.obstacle_avoidance_slam3d:main',
            'pwm_cvt = snuboat_nav90.pwm_converter:main',
        ],
    },
)
