import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'a24'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join('urdf', '*'))),
        # Include mesh files
        (os.path.join('share', package_name, 'meshes'),
         glob(os.path.join('meshes', '*'))),
        # Include rviz config files
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*'))),
        # Include world files
        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join('worlds', '*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='the-hassan-shahzad',
    maintainer_email='hshahzad2005108277@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
