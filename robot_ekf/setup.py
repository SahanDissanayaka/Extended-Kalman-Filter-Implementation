from setuptools import find_packages, setup

package_name = 'robot_ekf'

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
            'ekf_node = fyp_robot_ekf.ekf_node:main',
            'noisy_odom_node = fyp_robot_ekf.noisy_odom_node:main',
            'noisy_odom_path_node = fyp_robot_ekf.noisy_odom_path_node:main',
            'path_rmse_node = fyp_robot_ekf.path_rmse_node:main'
        ],
    },
)
