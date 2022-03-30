from setuptools import setup
import os #incluir
from glob import glob #incluir

package_name = 'automatix_escaneo_autonomo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruben',
    maintainer_email='rubenpardocasanova@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'escaneo_autonomo_server = automatix_escaneo_autonomo.escaneo_autonomo_server:main' 
        ],
    },
)
