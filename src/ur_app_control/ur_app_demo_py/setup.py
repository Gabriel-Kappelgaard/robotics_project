from glob import glob

from setuptools import find_packages, setup

package_name = 'ur_app_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabri',
    maintainer_email='gabriel@kappelgaard.com',
    description='Minimal moveit_py move demo for a UR5e',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'simple_move = ur_app_demo.simple_move:main',
        ],
    },
)
