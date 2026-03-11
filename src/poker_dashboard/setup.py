from setuptools import setup

package_name = 'poker_dashboard'

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
    description='Qt6 Dashboard for Poker Arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard = poker_dashboard.dashboard_node:main',
        ],
    },
)
