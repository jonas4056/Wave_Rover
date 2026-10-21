from setuptools import find_packages, setup

package_name = 'wave_rover_rc'

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
    description='Crossfire (CRSF) ontvanger naar sensor_msgs/Joy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'crsf_joy = wave_rover_rc.crsf_joy:main',
        ],
    },
)
