from setuptools import setup

package_name = 'keyboard_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/keyboard_controller.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gamepad_controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@local',
    description='Keyboard arrow key teleop publisher for motor control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_controller_node = keyboard_controller.keyboard_controller_node:main',
            'gamepad_controller_node = keyboard_controller.gamepad_controller_node:main',
        ],
    },
)
