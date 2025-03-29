from setuptools import setup
import os
from glob import glob

package_name = 'command_launcher'

setup(
    name=package_name,
    version='0.0.1',  # Increment the version for clarity
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # Ensure 'resource' folder exists and contains the package name
        ('share/' + package_name, ['package.xml']),  # Ensure 'package.xml' exists in the root directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))  # Use 'launch' subdirectory convention
    ],
    py_modules=[],  # This can be removed if you're not using standalone Python modules
    install_requires=['setuptools'],  # Dependency for setuptools
    zip_safe=True,
    maintainer='Li Bai',
    maintainer_email='your.email@example.com',  # Replace with the actual maintainer email
    description='A ROS 2 package to launch and stop nodes dynamically.',
    license='Apache License 2.0',  # Ensure you comply with this license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_launcher_node = command_launcher.command_launcher_node:main',  # Ensure the path matches the Python script structure
        ],
    },
)
