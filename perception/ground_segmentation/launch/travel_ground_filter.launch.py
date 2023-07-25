
import os

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
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]
        
    nodes = [
        ComposableNode(
            package="ground_segmentation",
            plugin="ground_segmentation::TravelGroundFilterComponent",
            name="travel_ground_filter",
            remappings=[
                ("input", LaunchConfiguration("input/pointcloud")),
                ("output", LaunchConfiguration("output/pointcloud")),
            ],
            parameters=[load_composable_node_param("travel_ground_filter_param_path")],
        ),
    ]

    loader = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals("container", ""),
        composable_node_descriptions=nodes,
        target_container=LaunchConfiguration("container"),
    )

    container = ComposableNodeContainer(
        name="travel_ground_filter_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
        condition=LaunchConfigurationEquals("container", ""),
    )

    group = GroupAction(
        [
            container,
            loader,
        ]
    )

    return [group]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    return launch.LaunchDescription(
        [
            add_launch_arg("container", ""),
            add_launch_arg("input/pointcloud", "pointcloud"),
            add_launch_arg("output/pointcloud", "no_ground/pointcloud"),
            add_launch_arg(
                "travel_ground_filter_param_path",
                [
                    FindPackageShare("autoware_launch"),
                    "/config/perception/obstacle_segmentation/ground_segmentation/travel_ground_filter.yaml",
                ],
            ),
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
