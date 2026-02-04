from setuptools import setup
import os
from glob import glob

package_name = 'poker_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- NEW: Install the models folder (CasADi files) ---
        ('share/' + package_name + '/models', glob('models/*.casadi')),
        
        # --- NEW: Install the config folder (YAML files) ---
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faisal',
    maintainer_email='faisallawan08@gmail.com',
    description='Poker Arm Control Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = poker_control.controller_node:main',
            'sim_bridge = poker_control.sim_bridge:main',
            'generate_kinematics = poker_control.generate_kinematics:main',
            'test_kinematics = poker_control.test_kinematics:main',
        ],
    },
)