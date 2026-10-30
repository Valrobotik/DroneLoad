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
    description='Vision : caméra, détection YOLO et flux vidéo vers le PC',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'camera = droneload_vision.camera:main',
            'vision = droneload_vision.vision:main',
            'gstream = droneload_vision.gstream:main',
        ],
    },
)
