from setuptools import setup
import os
from glob import glob

package_name = 'esp32_serial_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonas-nano',
    maintainer_email='',
    description='ESP32 Serial Bridge Node for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'esp32_serial_node = esp32_serial_bridge.esp32_serial_bridge_node:main',
        ],
    },
)

