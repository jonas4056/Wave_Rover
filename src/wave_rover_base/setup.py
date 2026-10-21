from setuptools import find_packages, setup

package_name = 'wave_rover_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonas De Clercq',
    maintainer_email='jonas_de_clercq49@hotmail.com',
    description='ESP32-bridge voor de Wave Rover (cmd_vel, IMU, batterij)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'esp32_bridge = wave_rover_base.esp32_bridge:main',
            'initialpose_relay = wave_rover_base.initialpose_relay:main',
            'lidar_odom_fix = wave_rover_base.lidar_odom_fix:main',
            'battery_monitor = wave_rover_base.battery_monitor:main',
        ],
    },
)
