from glob import glob

from setuptools import find_packages, setup


package_name = 'jetbot_aruco_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='MIPT JetBot team',
    maintainer_email='student@example.com',
    description='ArUco detection and exclusive autonomous following.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'http_camera_bridge = '
            'jetbot_aruco_follower.http_camera_bridge:main',
            'aruco_detector = jetbot_aruco_follower.detector_node:main',
            'aruco_follower = jetbot_aruco_follower.follower_node:main',
        ],
    },
)
