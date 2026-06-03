"""
RS2 Team 8 - Master launch file

map, waypoints_file, and params_file are ALL REQUIRED — the launch will
fail immediately with a clear error if any are missing. This prevents
accidentally running with wrong files on demo day.

Usage:
    # Simulation
    ros2 launch rs2_team8 rs2_tour.launch.py use_sim:=true \
        map:=$HOME/git/RS2Team8/r2_dTour/0_maps/simulation_map.yaml \
        waypoints_file:=$HOME/git/RS2Team8/r2_dTour/0_maps/simulation_waypoints.txt \
        params_file:=$HOME/git/RS2Team8/r2_dTour/rs2_team8/config/params/nav2_params.yaml

    # Real gallery robot
    ros2 launch rs2_team8 rs2_tour.launch.py use_sim:=false \
        map:=$HOME/git/RS2Team8/r2_dTour/0_maps/gallery_map.yaml \
        waypoints_file:=$HOME/git/RS2Team8/r2_dTour/0_maps/gallery_waypoints.txt \
        params_file:=$HOME/git/RS2Team8/r2_dTour/rs2_team8/config/params/nav2_params.yaml

Launch arguments (all required except use_sim):
    use_sim         (bool)    — true = Gazebo + no RViz. false = real robot + RViz.
    map             (string)  — absolute path to map .yaml file
    waypoints_file  (string)  — absolute path to waypoints .txt file
    params_file     (string)  — absolute path to nav2_params .yaml file

Sim:        Gazebo ON,  RViz OFF, initial pose set programmatically at (0,0)
Real robot: Gazebo OFF, RViz ON,  set pose manually via RViz 2D Pose Estimate

Real robot startup procedure:
    1. Run the launch command above with use_sim:=false
    2. Wait for RViz to open (~10 s for Nav2 to activate)
    3. In RViz click "2D Pose Estimate" and drag the green arrow to the
       robot's approximate position and heading — does not need to be exact
    4. Teleop one short lap to help AMCL converge before sending goals
    5. Once the particle cloud tightens in RViz, use the UI buttons
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def validate_args(context, *args, **kwargs):
    """Fail fast with a clear message if any required argument is missing."""
    errors = []

    map_val = context.launch_configurations.get("map", "")
    wp_val  = context.launch_configurations.get("waypoints_file", "")
    pf_val  = context.launch_configurations.get("params_file", "")

    if not map_val:
        errors.append("  map:=<path to .yaml>")
    elif not os.path.isfile(map_val):
        errors.append(f"  map: file not found: {map_val}")

    if not wp_val:
        errors.append("  waypoints_file:=<path to .txt>")
    elif not os.path.isfile(wp_val):
        errors.append(f"  waypoints_file: file not found: {wp_val}")

    if not pf_val:
        errors.append("  params_file:=<path to .yaml>")
    elif not os.path.isfile(pf_val):
        errors.append(f"  params_file: file not found: {pf_val}")

    if errors:
        msg = "\n\nRS2 Team 8 launch FAILED — missing or invalid arguments:\n"
        msg += "\n".join(errors)
        msg += "\n\nSee the docstring at the top of rs2_tour.launch.py for full commands.\n"
        sys.exit(msg)

    return []


def generate_launch_description():

    # -- Launch arguments -----------------------------------------------------
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="true = Gazebo sim (RViz off). false = real robot (RViz on).",
    )

    # No default_value — launch fails immediately if not supplied
    map_arg = DeclareLaunchArgument(
        "map",
        description="REQUIRED: absolute path to map .yaml file.",
    )

    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        description="REQUIRED: absolute path to waypoints .txt file.",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        description="REQUIRED: absolute path to nav2_params .yaml file.",
    )

    use_sim        = LaunchConfiguration("use_sim")
    map_yaml       = LaunchConfiguration("map")
    waypoints_file = LaunchConfiguration("waypoints_file")
    params_file    = LaunchConfiguration("params_file")

    # Validate all required args exist on disk before anything else starts
    validate = OpaqueFunction(function=validate_args)

    # -- 1. Gazebo (sim only) -------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("turtlebot3_gazebo"),
                "launch/turtlebot3_world.launch.py",
            )
        ),
        condition=IfCondition(use_sim),
    )

    # -- 2. Nav2 stack --------------------------------------------------------
    # Sim:        delayed 4 s for Gazebo to settle, RViz OFF
    # Real robot: delayed 2 s, RViz ON (operator must set 2D Pose Estimate)
    nav2_launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("turtlebot3_navigation2"),
                "launch/navigation2.launch.py",
            )
        ),
        launch_arguments={
            "map":         map_yaml,
            "params_file": params_file,
            "use_rviz":    "false",
        }.items(),
        condition=IfCondition(use_sim),
    )

    nav2_launch_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("turtlebot3_navigation2"),
                "launch/navigation2.launch.py",
            )
        ),
        launch_arguments={
            "map":         map_yaml,
            "params_file": params_file,
            "use_rviz":    "true",
        }.items(),
        condition=UnlessCondition(use_sim),
    )

    delayed_nav2_sim  = TimerAction(period=4.0, actions=[nav2_launch_sim])
    delayed_nav2_real = TimerAction(period=2.0, actions=[nav2_launch_real])

    # -- 3. navigation_node ---------------------------------------------------
    # Sim:        delayed 10 s — Nav2 fully activates then pose set at (0,0)
    # Real robot: delayed 15 s — gives operator time to set 2D Pose Estimate
    #             in RViz before waitUntilNav2Active() is called
    navigation_node_sim = Node(
        package="rs2_team8",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[{"waypoints_file": waypoints_file, "use_sim": True}],
        condition=IfCondition(use_sim),
    )

    navigation_node_real = Node(
        package="rs2_team8",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[{"waypoints_file": waypoints_file, "use_sim": False}],
        condition=UnlessCondition(use_sim),
    )

    delayed_navigation_node_sim  = TimerAction(period=10.0, actions=[navigation_node_sim])
    delayed_navigation_node_real = TimerAction(period=15.0, actions=[navigation_node_real])

    # -- 4. ui_node -----------------------------------------------------------
    # Two separate Node objects required — ros2 launch raises
    # "executed more than once" if the same instance appears twice.
    display = os.environ.get("DISPLAY", ":0")

    ui_node_sim = Node(
        package="rs2_team8",
        executable="ui_node",
        name="ui_node",
        output="screen",
        emulate_tty=True,
        additional_env={"DISPLAY": display},
        condition=IfCondition(use_sim),
    )

    ui_node_real = Node(
        package="rs2_team8",
        executable="ui_node",
        name="ui_node",
        output="screen",
        emulate_tty=True,
        additional_env={"DISPLAY": display},
        condition=UnlessCondition(use_sim),
    )

    delayed_ui_node_sim  = TimerAction(period=11.0, actions=[ui_node_sim])
    delayed_ui_node_real = TimerAction(period=16.0, actions=[ui_node_real])


    return LaunchDescription([
        use_sim_arg,
        map_arg,
        waypoints_file_arg,
        params_file_arg,
        validate,
        # Sim path
        gazebo_launch,
        delayed_nav2_sim,
        delayed_navigation_node_sim,
        delayed_ui_node_sim,
        # Real robot path
        delayed_nav2_real,
        delayed_navigation_node_real,
        delayed_ui_node_real,
    ])