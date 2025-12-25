# robot_gazebo_2.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- Launch args ---
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    description_format = LaunchConfiguration('description_format', default='urdf')

    # Default world inside your package
    default_world = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'), 'worlds', 'empty.world'
    ])
    world = LaunchConfiguration('world')

    # Declare launch args FIRST
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulated clock')
    declare_description_format = DeclareLaunchArgument(
        'description_format', default_value='urdf', description='urdf or sdf')
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world, description='Path to SDF world')

    # Now it's safe to reference `world` here
    gz_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                  'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': [' -r -v 1 ', world]}.items()
    )

    # --- robot_state_publisher (xacro -> robot_description) ---
    def robot_state_publisher(context):
        performed_description_format = description_format.perform(context)
        robot_description_content = Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'urdf',
                f'robot_velocity.xacro.{performed_description_format}'
            ]),
        ])
        robot_description = {
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        )]
    

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'config',
        'robot_velocity_controller.yaml',
    ])

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot',
            '-allow_renaming', 'true',
            '-x', '1.0', '-y', '0', '-z', '1'
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller', '--param-file', robot_controllers],
        output='screen',
    )

    leg_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_position_controller', '--param-file', robot_controllers],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )




    ld = LaunchDescription([
        # Start gz with your world (ONLY ONCE)

        declare_use_sim_time,
        declare_description_format,
        declare_world,

        gz_include,

        # Spawn robot then controllers in order
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=diff_drive_base_controller_spawner,
                on_exit=[leg_position_controller_spawner],
            )
        ),

        clock_bridge,
        gz_spawn_entity,

    ])


    # Add RSP after args are declared
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
