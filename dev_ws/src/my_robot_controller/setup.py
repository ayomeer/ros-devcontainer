from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ayommer@gmail.com',
    description='Robotics Backend Tutorial Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node_executable = my_robot_controller.my_first_node:main",
            "draw_circle_executable = my_robot_controller.draw_circle:main",
            "pose_subscriber_executable = my_robot_controller.pose_subscriber:main",
            "controller_executable = my_robot_controller.turtle_controller:main"
        ],
    },
)
