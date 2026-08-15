from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument("use_sim_time", default_value="false"),
    DeclareLaunchArgument("namespace", default_value=""),
    DeclareLaunchArgument(
        "params",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("turtlebot4_navigation"), "config", "localization.yaml"]
        ),
    ),
    DeclareLaunchArgument("map", description="Full path to map yaml file"),
    DeclareLaunchArgument("autostart", default_value="true"),
    DeclareLaunchArgument("bond_timeout", default_value="30.0"),
    DeclareLaunchArgument("lifecycle_start_delay", default_value="10.0"),
    DeclareLaunchArgument("log_level", default_value="info"),
]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace")
    params = LaunchConfiguration("params")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    bond_timeout = LaunchConfiguration("bond_timeout")
    lifecycle_start_delay = LaunchConfiguration("lifecycle_start_delay")
    log_level = LaunchConfiguration("log_level")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[configured_params, {"yaml_filename": map_yaml}],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            TimerAction(
                period=lifecycle_start_delay,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_localization",
                        output="screen",
                        arguments=["--ros-args", "--log-level", log_level],
                        parameters=[
                            {"autostart": ParameterValue(autostart, value_type=bool)},
                            {"node_names": ["map_server", "amcl"]},
                            {"bond_timeout": ParameterValue(bond_timeout, value_type=float)},
                        ],
                    )
                ],
            ),
        ]
    )
    return [nodes]


def generate_launch_description():
    description = LaunchDescription(ARGUMENTS)
    description.add_action(OpaqueFunction(function=launch_setup))
    return description
