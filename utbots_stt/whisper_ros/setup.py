from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'whisper_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ehg2004',
    maintainer_email='ehg2004@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_node = whisper_ros.whisper_node:main',
            'whisper_full_node = whisper_ros.whisper_full_node:main',
        ],
    },
    extras_require={
    'test': ['pytest', 'other-test-deps'],
    },

)
