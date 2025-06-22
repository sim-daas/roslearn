from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rostut'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/robolaunch.launch.py', 'launch/tanklaunch.launch.py']),
        ('share/' + package_name + '/urdf',
            ['urdf/robot.urdf.xacro', 'urdf/tank.urdf.xacro', 'urdf/robocore.urdf.xacro']),
        ('share/' + package_name + '/worlds',
            ['worlds/empty.world', 'worlds/houseoffice.sdf']),
        ('share/' + package_name + '/models',
            ['models/robot.sdf', 'models/tank.sdf']),
        ('share/' + package_name + '/models/shooter/meshes',
            ['models/shooter/meshes/barrel.stl', 'models/shooter/meshes/box.stl', 'models/shooter/meshes/nbarrel.stl']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='aitechmanml@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
