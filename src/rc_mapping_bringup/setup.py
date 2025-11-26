import os
from glob import glob

from setuptools import setup

package_name = 'rc_mapping_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'teleop_params.yaml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonas-nano',
    maintainer_email='jonas_de_clercq49@hotmail.com',
    description='RC mapping bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
