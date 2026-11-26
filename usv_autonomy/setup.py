from glob import glob
from setuptools import setup

setup(
    name='usv_autonomy', version='0.1.0', packages=['usv_autonomy'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/usv_autonomy']),
        ('share/usv_autonomy', ['package.xml']),
        ('share/usv_autonomy/launch', glob('launch/*.launch.py')),
        ('share/usv_autonomy/config', glob('config/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='USV team', maintainer_email='dev@usv.local',
    description='BT and lifecycle autonomy v2', license='Apache-2.0',
    entry_points={'console_scripts': [
        'mission = usv_autonomy.mission_node:main',
        'motion_gate = usv_autonomy.gate_node:main',
        'maritime_rules = usv_autonomy.rules_node:main',
        'authority_adapter = usv_autonomy.authority_node:main',
        'regular_safety = usv_autonomy.health_node:main',
        'simulation_controls = usv_autonomy.simulation_controls:main',
    ]},
)
