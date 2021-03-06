from setuptools import setup
import os #incluir
from glob import glob #incluir

package_name = 'automatix_nav_zona'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')) #incluir
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorelay',
    maintainer_email='rainbowg03@gmail.com',
    description='TODO: Packages description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_zona_server = automatix_nav_zona.nav_zona_server:main' #incluir
        ],
    },
)
