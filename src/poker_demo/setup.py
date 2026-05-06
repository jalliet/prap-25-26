from setuptools import setup

package_name = 'poker_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Faisal',
    maintainer_email='faisallawan08@gmail.com',
    description='Pick-and-flip demo sequences for the poker arm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pick_demo = poker_demo.pick_demo_node:main',
        ],
    },
)
