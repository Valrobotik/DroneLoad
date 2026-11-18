from glob import glob

from setuptools import setup

PACKAGE = 'droneload_bringup'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Lancement et paramètres du code embarqué DroneLoad',
    license='TODO: License declaration',
)
