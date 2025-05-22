from setuptools import setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    description='Sensor simulation nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = sensors.camera_node:main',
            'odom_node = sensors.odom_node:main',
            'imu_node = sensors.imu_node:main',
            'range_node = sensors.range_node:main',
        ],
    },
)

