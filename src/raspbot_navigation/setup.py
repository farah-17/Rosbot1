from glob import glob

from setuptools import setup

package_name = 'raspbot_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Raspbot V1: A* navigation on a saved Cartographer map, with a click-to-go GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_demo = raspbot_navigation.astar_demo:main',
            'navigator = raspbot_navigation.navigator:main',
            'nav_gui = raspbot_navigation.nav_gui:main',
            'sim_robot = raspbot_navigation.sim_robot:main',
            'set_start = raspbot_navigation.set_start:main',
            'coursework_nav = raspbot_navigation.coursework_nav:main',
        ],
    },
)
