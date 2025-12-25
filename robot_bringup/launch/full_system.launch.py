from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    gazebo_pkg = get_package_share_directory('robot_gazebo')
    path_pkg = get_package_share_directory('robot_path')
    ekf_pkg = get_package_share_directory('robot_ekf')

    # -------------------------
    # 1. Gazebo + robot + control
    # -------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                'launch',
                'robot_gazebo_velocity.launch.py'
            )
        )
    )

    # -------------------------
    # 2. Path controller
    # -------------------------
    path_controller = Node(
        package='robot_path',
        executable='path_controller',
        name='path_controller',
        output='screen'
    )

    noisy_odom = Node(
        package='robot_ekf',
        executable='noisy_odom_node',
        output='screen',
    )

    noisy_odom_path = Node(
        package='robot_ekf',
        executable='noisy_odom_path_node',  
        output='screen'
    )

    # -------------------------
    # 3. EKF
    # -------------------------
    ekf_node = Node(
        package='robot_ekf',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        remappings=[
            ('/odom', '/noisy_odom')
        ],
        parameters=[
            os.path.join(ekf_pkg, 'config', 'ekf_params.yaml')
        ]
    )

    # -------------------------
    # 4. Path visualizers
    # -------------------------
    ekf_path_node = Node(
        package='robot_path',
        executable='ekf_path_node',
        name='ekf_path_node',
        output='screen',
        remappings=[
        ('/odom', '/ekf_pose'),
        ('/cmd_vel', '/diff_drive_base_controller/cmd_vel')
        ]
    )

    odom_path_node = Node(
        package='robot_path',
        executable='odom_path_node',
        name='odom_path_node',
        output='screen',
        remappings=[
        ('/odom', '/diff_drive_base_controller/odom')
        ]
    )

    # -------------------------
    # 5. RViz
    # -------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    rmse_node = Node(
        package='robot_ekf',
        executable='path_rmse_node',
        output='screen'
    )


    clock_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    output='screen'
)


    return LaunchDescription([
        clock_bridge,
        gazebo_launch,
        path_controller,
        noisy_odom ,
        noisy_odom_path ,
        ekf_node,
        ekf_path_node,
        odom_path_node,
        rviz_node,
        rmse_node
    ])
