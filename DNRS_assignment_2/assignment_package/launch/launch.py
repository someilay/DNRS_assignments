from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    package_name = "assignment_package"
    package_path = FindPackageShare(package_name)
    model_path = PathJoinSubstitution(["urdf", "my_manipulator.urdf"])
    default_rviz_config_path = PathJoinSubstitution([package_path, "rviz", "urdf.rviz"])

    ld.add_action(
        DeclareLaunchArgument(
            name="gui",
            default_value="true",
            choices=["true", "false"],
            description="Flag to enable joint_state_publisher_gui",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="model",
            default_value=model_path,
            description="Path to robot urdf file relative to assignment_package package",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        )
    )

    rviz = GroupAction(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
                arguments=[PathJoinSubstitution([package_path, model_path])],
            ),
            Node(
                package=package_name,
                executable="move_manipulator",
                name="move_manipulator",
                output="screen",
            ),
        ]
    )
    ld.add_action(rviz)

    return ld
