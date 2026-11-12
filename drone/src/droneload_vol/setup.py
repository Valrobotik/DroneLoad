from setuptools import find_packages, setup

PACKAGE = 'droneload_vol'

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
    description='Vol : nœud principal, interface MAVROS et missions',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'main_node = droneload_vol.main_node:main',
        ],
    },
)
