from setuptools import find_packages, setup

package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/warehouse_simulation.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/warehouse_world.sdf']),
        ('share/' + package_name + '/models/diff_drive', [
            'models/diff_drive/model.config',
            'models/diff_drive/model.sdf'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ravali',
    maintainer_email='ravalimukkavilli@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_controller = warehouse_robot.robot_controller:main',
            'move_robot = warehouse_robot.move_robot:main',
        ],
    },
)
