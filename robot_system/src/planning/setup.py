from setuptools import setup

package_name = 'planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=['planning_nodes'],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='Planning package with generated service',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'global_planner = planning.global_planner:main',
            'lane_change_service = planning.lane_change_service:main',
            'local_trajectory = planning.local_trajectory:main',
        ],
    },
)

