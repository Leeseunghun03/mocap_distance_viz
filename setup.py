from glob import glob

from setuptools import setup

package_name = 'mocap_distance_viz'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsh12070420@gmail.com',
    maintainer_email='lsh12070420@gmail.com',
    description='Travelled-distance visualizer for mocap_optitrack rigid bodies',
    license='BSD',
    entry_points={
        'console_scripts': [
            'distance_viz_node = mocap_distance_viz.distance_viz_node:main',
            'fake_mocap_publisher = mocap_distance_viz.fake_mocap_publisher:main',
        ],
    },
)
