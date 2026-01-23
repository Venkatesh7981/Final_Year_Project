from setuptools import find_packages, setup

package_name = 'gesture_control_pkg'

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
    maintainer='suwi',
    maintainer_email='suwi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_tracker_publisher = gesture_control_pkg.hand_tracker_publisher:main',
            'gesture_joint_publisher = gesture_control_pkg.gesture_joint_publisher:main',
            'manual_angle_publisher = gesture_control_pkg.manual_angle_publisher:main',
        'angle_to_joint_state_publisher = gesture_control_pkg.angle_to_joint_state_publisher:main',        ],
    },
)
