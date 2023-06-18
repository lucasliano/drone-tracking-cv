from setuptools import setup

package_name = 'tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'keyboard',     # Used in joystick
    ],
    zip_safe=True,
    maintainer='Lucas Liaño',
    maintainer_email='lliano@frba.utn.edu.ar',
    description='ROS2 Node for PX4',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comms = tracker.comms:main',
            'joystick = tracker.joystick:main',
            'control = tracker.control_system:main',
        ],
    },
)
