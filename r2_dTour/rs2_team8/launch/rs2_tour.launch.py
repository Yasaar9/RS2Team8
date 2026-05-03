"""
RS2 Team 8 — Master launch file
Starts the full tour guide robot system in one command.

Usage:
    # Simulation (default)
    ros2 launch rs2_team8 rs2_tour.launch.py

    # Real gallery robot (skips Gazebo and RViz)
    ros2 launch rs2_team8 rs2_tour.launch.py use_sim:=false \
        waypoints_file:=$HOME/git/RS2Team8/r2_dTour/0_maps/gallery_waypoints.txt

Launch arguments:
    use_sim         (bool,   default true)  — launch Gazebo simulation
    waypoints_file  (string, default sim)   — absolute path to waypoints .txt file
    map             (string)                — absolute path to map .yaml file
    params_file     (string)                — absolute path to nav2_params .yaml file
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    home = os.path.expanduser("~")
    repo = os.path.join(home, "git/RS2Team8/r2_dTour")

    # ── Launch arguments ─────────────────────────────────────────────────────
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Launch Gazebo simulation. Set false for real robot.",
    )

    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=os.path.join(repo, "0_maps/simulation_waypoints.txt"),
        description="Absolute path to the waypoints .txt file.",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(repo, "0_maps/simulation_map.yaml"),
        description="Absolute path to the map .yaml file.",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(repo, "rs2_team8/config/params/nav2_params.yaml"),
        description="Absolute path to the nav2_params .yaml file.",
    )

    use_sim        = LaunchConfiguration("use_sim")
    waypoints_file = LaunchConfiguration("waypoints_file")
    map_yaml       = LaunchConfiguration("map")
    params_file    = LaunchConfiguration("params_file")

    # ── 1. Gazebo simulation (simulation only) ───────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("turtlebot3_gazebo"),
                "launch/turtlebot3_world.launch.py",
            )
        ),
        condition=IfCondition(use_sim),
    )

    # ── 2. Nav2 stack (no RViz) ──────────────────────────────────────────────
    # Delayed 4 s after Gazebo so the robot URDF and clock are available.
    # On real robot (use_sim=false) starts immediately.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("turtlebot3_navigation2"),
                "launch/navigation2.launch.py",
            )
        ),
        launch_arguments={
            "map":        map_yaml,
            "params_file": params_file,
            "use_rviz":   "false",   # skip RViz on both sim and real robot
        }.items(),
    )

    delayed_nav2 = TimerAction(period=4.0, actions=[nav2_launch])

    # ── 3. navigation_node ───────────────────────────────────────────────────
    # Delayed 10 s to give Nav2 time to fully activate before
    # waitUntilNav2Active() is called inside navigation_node.__init__.
    navigation_node = Node(
        package="rs2_team8",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[{"waypoints_file": waypoints_file}],
    )

    delayed_navigation_node = TimerAction(period=10.0, actions=[navigation_node])

    # ── 4. ui_node ──────────────────────────────────────────────────────────
    # Delayed 11 s — starts just after navigation_node.
    # DISPLAY must be forwarded explicitly so tkinter can open a window when
    # launched via ros2 launch (which does not always inherit the calling
    # terminal's display environment).
    import os as _os
    display = _os.environ.get("DISPLAY", ":0")

    ui_node = Node(
        package="rs2_team8",
        executable="ui_node",
        name="ui_node",
        output="screen",
        emulate_tty=True,
        additional_env={"DISPLAY": display},
    )

    delayed_ui_node = TimerAction(period=11.0, actions=[ui_node])

    return LaunchDescription([
        use_sim_arg,
        waypoints_file_arg,
        map_arg,
        params_file_arg,
        gazebo_launch,
        delayed_nav2,
        delayed_navigation_node,
        delayed_ui_node,
    ])