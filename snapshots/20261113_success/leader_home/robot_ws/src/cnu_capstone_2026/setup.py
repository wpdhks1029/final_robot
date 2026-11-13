from setuptools import setup

package_name = 'convoy_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'path_recorder = convoy_robot.path_recorder_node:main',
            'pure_pursuit  = convoy_robot.pure_pursuit_node:main',
        ],
    },
)
