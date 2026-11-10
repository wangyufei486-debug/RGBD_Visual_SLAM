from glob import glob

from setuptools import find_packages, setup

package_name = 'rgbd_slam_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wyf',
    maintainer_email='285603345+wangyufei486-debug@users.noreply.github.com',
    description='ROS 2 Jazzy RGB-D dataset publisher and RTAB-Map SLAM launch',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tum_rgbd_publisher = rgbd_slam_ros.tum_rgbd_publisher:main',
        ],
    },
)
