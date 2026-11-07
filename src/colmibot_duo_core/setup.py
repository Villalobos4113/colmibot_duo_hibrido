from glob import glob

from setuptools import find_packages, setup

package_name = 'colmibot_duo_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo COLMIBOT DUO (mantenedor por confirmar)',
    maintainer_email='pendiente@example.invalid',
    description='Núcleo local de COLMIBOT DUO: misión simbólica y evidencia SQLite.',
    license='TODO: licencia pendiente de decisión del equipo',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mission_node = colmibot_duo_core.adapters.ros_node:main',
        ],
    },
)
