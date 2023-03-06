from setuptools import setup
import os
from glob import glob

package_name = 'training_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),

        (os.path.join('share', package_name, 'config/simulation'), 
            glob(os.path.join('config', 'simulation', '*.yaml'))),

        (os.path.join('share', package_name, 'config/physical'),
            glob(os.path.join('config', 'physical', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mroglan',
    maintainer_email='manueljoseph113@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'republisher = training_localization.republisher_node:main'
        ],
    },
)
