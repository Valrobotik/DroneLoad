from setuptools import find_packages, setup

PACKAGE = 'droneload_vision'

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
    description="Vision : détection YOLO et ArUco avec pose 3D (traitement d'image, sans matériel)",
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'vision = droneload_vision.vision:main',
            'aruco = droneload_vision.aruco:main',
        ],
    },
)
