from glob import glob

from setuptools import find_packages, setup

package_name = 'jetbot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='app',
    maintainer_email='obulichev@yandex.ru',
    description='JetBot sensor and hardware bringup package.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = jetbot_bringup.keyboard_teleop:main',
        ],
    },
)
