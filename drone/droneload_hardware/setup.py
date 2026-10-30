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
    description='Matériel et liaisons : servo du crochet, serveur MQTT (dashboard, ESP32)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'servo = droneload_hardware.servo:main',
            'mqtt_server = droneload_hardware.mqtt_server:main',
        ],
    },
)
