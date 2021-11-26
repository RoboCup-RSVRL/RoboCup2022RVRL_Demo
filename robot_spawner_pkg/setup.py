from setuptools import setup

setup(
    name='robot_spawner_pkg',
    version='1.0.0',
    package_dir={'': 'src'},
    packages=['robot_spawner_pkg'],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_robot = robot_spawner_pkg.spawn_robot:main',
        ],
    },
    data_files=[
        ('share/robot_spawner_pkg' , ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/robot_spawner_pkg']),
    ],
)