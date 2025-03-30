from setuptools import setup
import os
from glob import glob

package_name = 'servo_toggle_client'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Client for toggling Mini Pupper servos',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_toggle = servo_toggle_client.servo_toggle_client:main',
            'servo_toggle_node = servo_toggle_client.nodes.servo_toggle_client:main'
        ],
    },
)