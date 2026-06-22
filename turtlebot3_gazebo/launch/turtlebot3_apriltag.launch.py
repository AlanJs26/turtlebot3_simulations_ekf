#!/usr/bin/env python3
import os
import math
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_entities(context, *args, **kwargs):
    # Evaluate launch configurations
    camera_height_str = context.launch_configurations.get("camera_height", "2.5")
    try:
        camera_height = float(camera_height_str)
    except ValueError:
        camera_height = 2.5

    # 1. Geometry Calculations
    # Camera horizontal FOV is 60 degrees (1.04719755 rad)
    fov_h = 1.04719755
    aspect_ratio = 16.0 / 9.0

    # width of view = 2 * H * tan(fov / 2)
    W = 2.0 * camera_height * math.tan(fov_h / 2.0)
    H_v = W / aspect_ratio

    # Inset from camera view edges to make sure the 10cm tags are fully visible
    # Tag size is 0.1m, so we use a margin of 0.15m (half tag size + 0.1m buffer)
    margin = 0.15
    x_max = W / 2.0 - margin
    y_max = H_v / 2.0 - margin

    print(f"[AprilTag Sim] Camera Height: {camera_height}m")
    print(f"[AprilTag Sim] Calculated FOV area width: {W:.3f}m, height: {H_v:.3f}m")
    print(
        f"[AprilTag Sim] Inset tag coordinates: x_max = {x_max:.3f}m, y_max = {y_max:.3f}m"
    )

    # 2. Update config file (tags.yaml) in the workspace to synchronize tracker calibration
    config_path = (
        "/root/turtlebot3_ws/workspace/turtlebot3_apriltag_rerun/config/tags.yaml"
    )
    if os.path.exists(config_path):
        try:
            with open(config_path, "r") as f:
                yaml_data = yaml.safe_load(f) or {}

            # Support ROS 2 parameters format
            if "/**" not in yaml_data:
                yaml_data["/**"] = {}
            if "ros__parameters" not in yaml_data["/**"]:
                yaml_data["/**"]["ros__parameters"] = {}
            
            params = yaml_data["/**"]["ros__parameters"]
            params["boundary_ids"] = [0, 1, 2, 4]
            params["boundary_positions"] = [
                float(-x_max), float(-y_max),
                float(x_max), float(-y_max),
                float(x_max), float(y_max),
                float(-x_max), float(y_max)
            ]

            with open(config_path, "w") as f:
                yaml.safe_dump(yaml_data, f, default_flow_style=False)
            print(
                f"[AprilTag Sim] Successfully updated {config_path} with calibrated coordinates."
            )
        except Exception as e:
            print(f"[AprilTag Sim] Warning: Could not update tags.yaml: {e}")
    else:
        print(f"[AprilTag Sim] Warning: config path {config_path} not found.")

    pkg_share = get_package_share_directory("turtlebot3_gazebo")
    spawn_actions = []

    # 3. Spawn Aerial Camera
    # Rotations: Roll = 0.0, Pitch = 90 degrees down (1.570796), Yaw = 180 degrees (3.141592)
    spawn_actions.append(
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity",
                "aerial_camera",
                "-file",
                os.path.join(pkg_share, "models", "aerial_camera", "model.sdf"),
                "-x",
                "0.0",
                "-y",
                "0.0",
                "-z",
                str(camera_height),
                "-R",
                "0.0",
                "-P",
                "1.570796",
                "-Y",
                "1.570796",
            ],
            output="screen",
        )
    )

    # 4. Spawn AprilTags on the ground
    tags = [
        ("apriltag_0", -x_max, -y_max),
        ("apriltag_1", x_max, -y_max),
        ("apriltag_2", x_max, y_max),
        ("apriltag_4", -x_max, y_max),
    ]

    for tag_name, tx, ty in tags:
        spawn_actions.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    tag_name,
                    "-file",
                    os.path.join(pkg_share, "models", tag_name, "model.sdf"),
                    "-x",
                    str(tx),
                    "-y",
                    str(ty),
                    "-z",
                    "0.001",
                ],
                output="screen",
            )
        )

    # 5. Spawn TurtleBot3 Burger
    TURTLEBOT3_MODEL = os.environ.get("TURTLEBOT3_MODEL", "burger")
    model_folder = "turtlebot3_" + TURTLEBOT3_MODEL
    urdf_path = os.path.join(pkg_share, "models", model_folder, "model.sdf")

    spawn_actions.append(
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity",
                TURTLEBOT3_MODEL,
                "-file",
                urdf_path,
                "-x",
                "0.0",
                "-y",
                "0.0",
                "-z",
                "0.01",
            ],
            output="screen",
        )
    )

    return spawn_actions


def generate_launch_description():
    pkg_share = get_package_share_directory("turtlebot3_gazebo")
    launch_file_dir = os.path.join(pkg_share, "launch")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    world = os.path.join(pkg_share, "worlds", "turtlebot3_apriltag.world")

    gazebo_plugin_path = str(
        Path(pkg_share).parent.parent / "lib" / "turtlebot3_gazebo"
    )
    environment_variable_cmd = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH", value=gazebo_plugin_path
    )

    # Launch gzserver with our custom world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    # Launch gzclient
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    # Robot State Publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Declare camera height launch argument
    declare_camera_height_cmd = DeclareLaunchArgument(
        "camera_height",
        default_value="2.5",
        description="Height of the overhead camera",
    )

    ld = LaunchDescription()

    # Add commands
    ld.add_action(environment_variable_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(declare_camera_height_cmd)

    # Spawn entities using OpaqueFunction to parse camera_height
    ld.add_action(OpaqueFunction(function=spawn_entities))

    return ld
