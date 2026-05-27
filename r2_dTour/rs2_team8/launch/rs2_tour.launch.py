"""
RS2 Team 8 — Master launch file
Starts the full tour guide robot system in one command.

Usage:
    # Simulation (default)
    ros2 launch rs2_team8 rs2_tour.launch.py

    # Real gallery robot
    ros2 launch rs2_team8 rs2_tour.launch.py use_sim:=false

    # Real gallery robot with explicit paths (if defaults don't match your setup)
    ros2 launch rs2_team8 rs2_tour.launch.py use_sim:=false \
        map:=$HOME/git/RS2Team8/r2_dTour/0_maps/gallery_map.yaml \
        waypoints_file:=$HOME/git/RS2Team8/r2_dTour/0_maps/gallery_waypoints.txt

Launch arguments:
    use_sim         (bool,   default true)  — launch Gazebo + use sim map/waypoints
    waypoints_file  (string)                — override waypoints .txt path
    map             (string)                — override map .yaml path
    params_file     (string)                — override nav2_params .yaml path

Real robot startup procedure:
    1. ros2 launch rs2_team8 rs2_tour.launch.py use_sim:=false
    2. Wait for RViz to open (takes ~10 s for Nav2 to activate).
    3. In RViz, click "2D Pose Estimate" and drag the green arrow to the
       robot's approximate position and heading on the map. Does not need
       to be exact — AMCL's particle cloud will converge from here.
    4. Teleop the robot one short lap (or just drive it forward and back)
       to help AMCL converge before sending navigation goals.
    5. Once the pose cloud has converged in RViz, press buttons in the UI.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    home = os.path.expanduser("~")
    repo = os.path.join(home, "git/RS2Team8/r2_dTour")

    # ── Launch arguments ─────────────────────────────────────────────────────
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="true = Gazebo sim (default). false = real gallery robot.",
    )

    # Waypoints: sim default vs gallery default, overridable
    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=PythonExpression([
            "'", os.path.join(repo, "0_maps/gallery_waypoints.txt"), "' if '",
            LaunchConfiguration("use_sim"),
            "' == 'false' else '",
            os.path.join(repo, "0_maps/simulation_waypoints.txt"), "'",
        ]),
        description="Absolute path to the waypoints .txt file.",
    )

    # Map: sim default vs gallery default, overridable
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=PythonExpression([
            "'", os.path.join(repo, "0_maps/gallery_map.yaml"), "' if '",
            LaunchConfiguration("use_sim"),
            "' == 'false' else '",
            os.path.join(repo, "0_maps/simulation_map.yaml"), "'",
        ]),
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

    # ── 2. Nav2 stack ─────────────────────────────────────────────────────────
    # Simulation : delayed 4 s so Gazebo clock and URDF are available first.
    # Real robot : starts immediately (no Gazebo to wait for).
    #
    # use_rviz:
    #   Simulation  → false  (no need for RViz; initial pose set programmatically)
    #   Real robot  → true   (operator MUST use RViz 2D Pose Estimate to set pose)
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
            "use_rviz":    "true",   # RViz required on real robot for 2D Pose Estimate
        }.items(),
        condition=UnlessCondition(use_sim),
    )

    delayed_nav2_sim  = TimerAction(period=4.0, actions=[nav2_launch_sim])
    # Real robot Nav2 starts immediately — no Gazebo delay needed
    delayed_nav2_real = TimerAction(period=2.0, actions=[nav2_launch_real])

    # ── 3. navigation_node ───────────────────────────────────────────────────
    # Delayed 10 s (sim) / 15 s (real) to give Nav2 time to fully activate.
    # Real robot gets extra time because the operator also needs to set the
    # 2D Pose Estimate in RViz before navigation_node calls waitUntilNav2Active().
    # use_sim is passed as a parameter so navigation_node skips setInitialPose()
    # on the real robot (pose is set manually via RViz instead).
    navigation_node_sim = Node(
        package="rs2_team8",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[{
            "waypoints_file": waypoints_file,
            "use_sim": True,
        }],
        condition=IfCondition(use_sim),
    )

    navigation_node_real = Node(
        package="rs2_team8",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[{
            "waypoints_file": waypoints_file,
            "use_sim": False,
        }],
        condition=UnlessCondition(use_sim),
    )

    delayed_navigation_node_sim  = TimerAction(period=10.0, actions=[navigation_node_sim])
    delayed_navigation_node_real = TimerAction(period=15.0, actions=[navigation_node_real])

    # ── 4. ui_node ──────────────────────────────────────────────────────────
    # Delayed 1 s after navigation_node in each case.
    # DISPLAY forwarded explicitly so tkinter opens a window under ros2 launch.
    display = os.environ.get("DISPLAY", ":0")

    ui_node = Node(
        package="rs2_team8",
        executable="ui_node",
        name="ui_node",
        output="screen",
        emulate_tty=True,
        additional_env={"DISPLAY": display},
    )

    delayed_ui_node_sim  = TimerAction(period=11.0, actions=[ui_node])
    delayed_ui_node_real = TimerAction(period=16.0, actions=[ui_node])

    # detector_node = Node(
    #     package="rs2_team8",
    #     executable="detector_node",
    #     name="detector_node",
    #     output="screen"
    # )
    # delayed_detector_node = TimerAction(period=12.0, actions=[detector_node])

    return LaunchDescription([
        use_sim_arg,
        waypoints_file_arg,
        map_arg,
        params_file_arg,
        # Sim path
        gazebo_launch,
        delayed_nav2_sim,
        delayed_navigation_node_sim,
        delayed_ui_node_sim,
        # Real robot path
        delayed_nav2_real,
        delayed_navigation_node_real,
        delayed_ui_node_real,
        # delayed_detector_node
    ])