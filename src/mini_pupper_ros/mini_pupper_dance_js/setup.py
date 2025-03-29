from setuptools import setup
import os
from glob import glob

package_name = 'mini_pupper_dance_js'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[
        f'{package_name}.mini_pupper_dance_js',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 subscriber node to execute Mini Pupper dance files.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mini_pupper_dance_js = mini_pupper_dance_js.mini_pupper_dance_js:main',
        ],
    },
)
