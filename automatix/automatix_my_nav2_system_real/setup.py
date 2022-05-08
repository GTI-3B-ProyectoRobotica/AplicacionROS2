import os
from glob import glob
from setuptools import setup


package_name = 'automatix_my_nav2_system_real'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.pgm')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.model'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asun',
    maintainer_email='asun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = automatix_my_nav2_system_real.initial_pose_pub:main',
            'navigate_to_pose_client = automatix_my_nav2_system_real.navigate_to_pose_client:main',
            'load_map = automatix_my_nav2_system_real.load_map:main',
            'nav_to_pose = automatix_my_nav2_system_real.nav_to_pose:main',
            'go_to_pose = automatix_my_nav2_system_real.go_to_pose:main'
        ],
    },
)
