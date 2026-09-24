import os
import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package paths
    package_share = get_package_share_directory('pointcloud_bot')

    world_file = os.path.join(
        package_share,
        'worlds',
        'room_world.sdf'
    )

    robot_description_file = os.path.join(
        package_share,
        'urdf',
        'robot.urdf.xacro'
    )

    models_path = os.path.join(
        package_share,
        'models'
    )

    # Process Xacro → URDF
    robot_description = xacro.process_file(
        robot_description_file
    ).toxml()

    # Make Gazebo aware of our custom models
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    # Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 "{world_file}"'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={os.path.join(package_share, "config", "bridge.yaml")}'
        ],
        output='screen'
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'pointcloud_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.25'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])