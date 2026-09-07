from setuptools import setup

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/six_axis_arm.urdf']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/launch', ['launch/autonomous.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/display.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahimkhan',
    maintainer_email='ibrahimkhan@todo.todo',
    description='6-axis robot arm kinematics and path planning',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_node = robot_arm.fk_node:main',
            'ik_node = robot_arm.ik_node:main',
            'driver_node = robot_arm.driver_node:main',
            'trail_node = robot_arm.trail_node:main',
            'obstacle_node = robot_arm.obstacle_node:main',
            'planner_node = robot_arm.planner_node:main',
        ],
    },
)