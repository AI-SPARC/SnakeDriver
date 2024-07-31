from setuptools import find_packages, setup

package_name = 'snake5r_driver'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch',
        ['launch/snake.launch.py']),
    ('share/' + package_name + '/rviz',
        ['rviz/config.rviz']),
    ('share/' + package_name + '/rviz',
        ['rviz/config.rviz']),
    ('share/' + package_name + '/meshes',
        ['meshes/lx16a_servo_socket.dae', 'meshes/lx16a_servo.dae', 'meshes/lx16a_socket.dae',
        'meshes/servo.stl', 'meshes/servoSocket.stl', 'meshes/socket.stl']),
    ('share/' + package_name + '/urdf',
        ['urdf/snake_robot.urdf', 'urdf/snake_robot.xacro']),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='robotica@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
