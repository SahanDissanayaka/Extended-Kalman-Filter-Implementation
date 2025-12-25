from setuptools import find_packages, setup

package_name = 'robot_path'

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
    maintainer='sahan-dissanayaka',
    maintainer_email='sahanlive984@gmail.com',
    description='TODO: Package description',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_path_node = fyp_robot_path.odom_path_node:main',
            'ekf_path_node = fyp_robot_path.ekf_path_node:main',
            'path_controller = fyp_robot_path.path_controller:main',
            'test_odom = fyp_robot_path.test_odom:main'
        ],
    },
)
