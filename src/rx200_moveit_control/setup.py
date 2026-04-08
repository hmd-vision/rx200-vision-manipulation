from setuptools import find_packages, setup

package_name = 'rx200_moveit_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_all.launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'launch', 'launch_ros', 'cv2'],
    zip_safe=True,
    maintainer='James Florin Petri',
    maintainer_email='james.petri@mu.ie',
    description='Package used for RX200 arm control via the moveit interface',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rx200_moveit_client=rx200_moveit_control.rx200_moveit_action_client:main',
            'keyboard_gui=rx200_moveit_control.gui_publisher:main',
            'vision=rx200_moveit_control.vision_subscriber:main'
        ],
    },
)
