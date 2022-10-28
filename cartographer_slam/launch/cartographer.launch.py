from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "package_name",
            default_value="cartographer_slam",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "configuration_basename",
            default_value="cartographer.lua",
        )
    )

    package_name = LaunchConfiguration("package_name")
    configuration_basename = LaunchConfiguration("configuration_basename")

    cartographer_config_dir = PathJoinSubstitution(
        [FindPackageShare(package_name), "config"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "rviz.rviz"]
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
