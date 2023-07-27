import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def launch_setup(context, *args, **kwargs):
    travel_filter_param_path = LaunchConfiguration(
        "travel_ground_filter_param_path"
    ).perform(context)
    with open(travel_filter_param_path, "r") as f:
        travel_filter_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    travel_filter_param = travel_filter_param["common_ground_filter"]["parameters"]

    nodes = [
        ComposableNode(
            package="ground_segmentation",
            plugin="ground_segmentation::TravelGroundFilterComponent",
            name="travel_ground_filter",
            remappings=[
                ("input", LaunchConfiguration("input/pointcloud")),
                ("output", LaunchConfiguration("output/pointcloud")),
            ],
            parameters=[travel_filter_param],
        ),
    ]

    # loader = LoadComposableNodes(
    #     condition=LaunchConfigurationNotEquals("container", ""),
    #     composable_node_descriptions=nodes,
    #     target_container=LaunchConfiguration("container"),
    # )

    container = ComposableNodeContainer(
        name="travel_ground_filter_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
        # condition=LaunchConfigurationEquals("container", ""),
    )

    group = GroupAction(
        [
            container,
            # loader,
        ]
    )

    return [group]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    default_travel_filter_param_path = os.path.join(
        get_package_share_directory(
            "ground_segmentation"
        ),  # autoware_launch'a tasiniyordu en son
        "config/travel_ground_filter.param.yaml",
    )

    travel_ground_filter_param = DeclareLaunchArgument(
        "travel_ground_filter_param_path",
        default_value=default_travel_filter_param_path,
        description="Path to config file for travel_ground_filter information",
    )

    return launch.LaunchDescription(
        [
            travel_ground_filter_param,
            add_launch_arg("container", ""),
            add_launch_arg(
                "input/pointcloud", "/sensing/lidar/concatenated/pointcloud"
            ),
            add_launch_arg("output/pointcloud", "nonground"),
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
