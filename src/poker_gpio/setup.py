from setuptools import setup

package_name = 'poker_gpio'

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
    description='Raspberry Pi GPIO bridge for the poker pump and button.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'poker_gpio = poker_gpio.poker_gpio_node:main',
        ],
    },
)
