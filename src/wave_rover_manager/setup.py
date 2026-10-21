from setuptools import find_packages, setup

package_name = 'wave_rover_manager'

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
    description='Start en stopt modules van de Wave Rover op vraag',
    license='MIT',
    entry_points={
        'console_scripts': [
            'module_manager = wave_rover_manager.module_manager:main',
        ],
    },
)
