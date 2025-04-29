from setuptools import setup

package_name = 'smallrobot_alex'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # No need to explicitly list 'action' here
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/action', ['action/PlanMovement.action']) # Keep this for installation
    ],
    install_requires=['setuptools', 'rclpy', 'rclpy_action', 'moveit_configs_utils', 'moveit_commander'],
    zip_safe=True,
    maintainer='...',
    maintainer_email='...',
    description='...',
    license='...',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_action_server = smallrobot_alex.planner_action_server:main',
        ],
    },
)