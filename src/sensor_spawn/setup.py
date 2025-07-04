from setuptools import setup

package_name = 'sensor_spawn'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/sensor_spawn/launch', ['launch/sensor_nodes.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dyllon Dunton',
    maintainer_email='duntondyllon@gmail.com',
    description='Start up LiDAR and camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
