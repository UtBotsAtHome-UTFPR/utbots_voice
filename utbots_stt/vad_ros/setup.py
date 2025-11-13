from setuptools import setup,find_packages
import os
from glob import glob

package_name = 'vad_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
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
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'vad_node = vad_ros.vad_node:main'
        ],
    },
    extras_require={
    'test': ['pytest', 'other-test-deps'],
    },
)
