from setuptools import find_packages, setup

PACKAGE = 'droneload_hardware'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Matériel et liaisons : caméra, servo du crochet, flux vidéo vers le PC, serveur MQTT (dashboard, ESP32), position UWB vers la Pixhawk',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'camera = droneload_hardware.camera:main',
            'servo = droneload_hardware.servo:main',
            'gstream = droneload_hardware.gstream:main',
            'mqtt_server = droneload_hardware.mqtt_server:main',
            'position_ext = droneload_hardware.position_ext:main',
            'uwb = droneload_hardware.uwb:main',
        ],
    },
)
