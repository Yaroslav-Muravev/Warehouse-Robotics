from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    description='Fiducial detection and localization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fiducial_detector = perception.fiducial_detector:main',
            'localization_node = perception.localization_node:main',
        ],
    },
)

