from setuptools import setup
import os #incluir
from glob import glob #incluir

package_name = 'automatix_guardar_zona_service'

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
    maintainer='pablo',
    maintainer_email='pabloenguixllopis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'automatix_guardar_zona_server = automatix_guardar_zona_service.automatix_guardar_zona_server:main', #incluir
            'automatix_guardar_zona_client = automatix_guardar_zona_service.automatix_guardar_zona_client:main', #incluir

            'automatix_borrar_zona_server = automatix_guardar_zona_service.automatix_borrar_zona_server:main', #incluir
            'automatix_borrar_zona_client = automatix_guardar_zona_service.automatix_borrar_zona_client:main' #incluir
        ],
    },
)
