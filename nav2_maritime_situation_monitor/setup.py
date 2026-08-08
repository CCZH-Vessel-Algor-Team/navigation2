from glob import glob
import os

from setuptools import setup


package_name = 'nav2_maritime_situation_monitor'

setup(
    name=package_name,
    version='1.1.20',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vectorwang',
    maintainer_email='vectorwang@hotmail.com',
    description='Publishes maritime CPA, risk, and encounter situation reports.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maritime_situation_monitor = '
            'nav2_maritime_situation_monitor.situation_monitor_node:main',
        ],
    },
)
