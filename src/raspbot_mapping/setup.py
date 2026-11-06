from glob import glob

from setuptools import setup

package_name = 'raspbot_mapping'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Raspbot V1 + YDLIDAR 4ROS lidar mapping',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_driver = raspbot_mapping.motor_driver:main',
            'motor_test = raspbot_mapping.motor_test:main',
            'scan_check = raspbot_mapping.scan_check:main',
        ],
    },
)
