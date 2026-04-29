# RS2Team8
Robotics Studio 2 - Social Tour Guide Robo


## Install ROS 2

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

## Dependicies

TurtleBot3 e-Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

sudo apt install python3-pip
sudo apt install mpg123
pip3 install gtts

## Create git folder and clone

mkdir -p ~/git
cd ~/git
git clone git@github.com:Yasaar9/RS2Team8.git

## Create workspace and link

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
ln -s ~/git/RS2Team8/r2_dTour

## Build and configure

cd ~/turtlebot3_ws
colcon build --symlink-install --cmake-args -Wno-dev
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc


## Rebuild fresh

cd ~/turtlebot3_ws
rm -rf build install log
colcon build --symlink-install --cmake-args -Wno-dev
source ~/.bashrc

## Run Simulation

### Permanently Select Turtlebot Model

echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc

### Empty World

ros2 launch turtlebot3_gazebo empty_world.launch.py

### World

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

### House

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

### Start Nav2

ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=$HOME/map.yaml \
  params_file:=$HOME/git/RS2Team8/r2_dTour/RS2Team8_Package/config/params/nav2_params.yaml

### Start Navigation node

ros2 run RS2Team8_Package navigation_node

### Start Navigation code

ros2 topic pub --once /navigation/go_to_waypoint std_msgs/String "data: 'artifact_1'"


## FOLDER STRUCTURE
================================================================================

~/git/RS2Team8/                    ← Git repository (commit/push from here)
├── README.md                      ← Setup instructions for teammates
├── .gitignore                     ← Excludes build/, install/, log/
└── r2_dTour/                      ← All packages (dependencies + your code)
    ├── DynamixelSDK/              ← Real files (cloned from ROBOTIS)
    ├── turtlebot3_msgs/           ← Real files (cloned from ROBOTIS)
    ├── turtlebot3/                ← Real files (cloned from ROBOTIS)
    ├── turtlebot3_simulations/    ← Real files (cloned from ROBOTIS)
    └── RS2Team8_Package/            ← Your custom packages here
        ├── package.xml
        ├── setup.py
        ├── launch/
        ├── config/
        │   ├── maps/              ← .pgm and .yaml map files
        │   ├── rviz/              ← .rviz config files
        │   └── params/            ← .yaml parameter files
        ├── worlds/                ← .world Gazebo world files
        ├── urdf/                  ← Robot description files
        └── RS2Team8_Package/        ← Python module
            ├── __init__.py
            └── nodes/             ← Your ROS nodes

~/turtlebot3_ws/                   ← Build workspace (not in git)
└── src/
    └── r2_dTour -> /home/jsunne/git/RS2Team8/r2_dTour   ← SYMLINK

## Rose Nodes

ros2 node list
/gazebo
/robot_state_publisher
/turtlebot3_diff_drive
/turtlebot3_imu
/turtlebot3_joint_state
/turtlebot3_laserscan


# List packages in your workspace only
ros2 pkg list | grep -v /opt/ros
RS2Team8_Package
ackermann_msgs
action_msgs
action_tutorials_cpp
action_tutorials_interfaces
action_tutorials_py
actionlib_msgs
ament_cmake
ament_cmake_auto
ament_cmake_copyright
ament_cmake_core
ament_cmake_cppcheck
ament_cmake_cpplint
ament_cmake_export_definitions
ament_cmake_export_dependencies
ament_cmake_export_include_directories
ament_cmake_export_interfaces
ament_cmake_export_libraries
ament_cmake_export_link_flags
ament_cmake_export_targets
ament_cmake_flake8
ament_cmake_gen_version_h
ament_cmake_gmock
ament_cmake_gtest
ament_cmake_include_directories
ament_cmake_libraries
ament_cmake_lint_cmake
ament_cmake_pep257
ament_cmake_pytest
ament_cmake_python
ament_cmake_ros
ament_cmake_target_dependencies
ament_cmake_test
ament_cmake_uncrustify
ament_cmake_version
ament_cmake_xmllint
ament_copyright
ament_cppcheck
ament_cpplint
ament_flake8
ament_index_cpp
ament_index_python
ament_lint
ament_lint_auto
ament_lint_cmake
ament_lint_common
ament_package
ament_pep257
ament_uncrustify
ament_xmllint
angles
behaviortree_cpp_v3
bond
bondcpp
builtin_interfaces
camera_calibration_parsers
camera_info_manager
cartographer_ros
cartographer_ros_msgs
class_loader
common_interfaces
composition
composition_interfaces
console_bridge_vendor
control_msgs
control_toolbox
controller_interface
controller_manager
controller_manager_msgs
costmap_queue
cv_bridge
demo_nodes_cpp
demo_nodes_cpp_native
demo_nodes_py
depthimage_to_laserscan
desktop
diagnostic_msgs
diagnostic_updater
diff_drive_controller
domain_coordinator
dummy_map_server
dummy_robot_bringup
dummy_sensors
dwb_core
dwb_critics
dwb_msgs
dwb_plugins
dynamixel_sdk
dynamixel_sdk_custom_interfaces
dynamixel_sdk_examples
effort_controllers
eigen3_cmake_module
example_interfaces
examples_rclcpp_minimal_action_client
examples_rclcpp_minimal_action_server
examples_rclcpp_minimal_client
examples_rclcpp_minimal_composition
examples_rclcpp_minimal_publisher
examples_rclcpp_minimal_service
examples_rclcpp_minimal_subscriber
examples_rclcpp_minimal_timer
examples_rclcpp_multithreaded_executor
examples_rclpy_executors
examples_rclpy_minimal_action_client
examples_rclpy_minimal_action_server
examples_rclpy_minimal_client
examples_rclpy_minimal_publisher
examples_rclpy_minimal_service
examples_rclpy_minimal_subscriber
fastrtps_cmake_module
filters
forward_command_controller
gazebo_dev
gazebo_model_attachment_plugin
gazebo_model_attachment_plugin_msgs
gazebo_msgs
gazebo_no_physics_plugin
gazebo_planar_move_plugin
gazebo_plugins
gazebo_ros
gazebo_ros2_control
gazebo_ros2_control_demos
gazebo_ros_pkgs
gazebo_set_joint_positions_plugin
gazebo_video_monitor_interfaces
gazebo_video_monitor_plugins
gazebo_video_monitor_utils
gazebo_video_monitors
generate_parameter_library
generate_parameter_library_py
geometry2
geometry_msgs
hardware_interface
image_geometry
image_tools
image_transport
imu_sensor_broadcaster
interactive_markers
intra_process_demo
joint_limits
joint_state_broadcaster
joint_trajectory_controller
joy
kdl_parser
keyboard_handler
laser_geometry
launch
launch_ros
launch_testing
launch_testing_ament_cmake
launch_testing_ros
launch_xml
launch_yaml
libcurl_vendor
libstatistics_collector
libyaml_vendor
lifecycle
lifecycle_msgs
logging_demo
map_msgs
message_filters
nav2_amcl
nav2_behavior_tree
nav2_behaviors
nav2_bringup
nav2_bt_navigator
nav2_collision_monitor
nav2_common
nav2_constrained_smoother
nav2_controller
nav2_core
nav2_costmap_2d
nav2_dwb_controller
nav2_lifecycle_manager
nav2_map_server
nav2_mppi_controller
nav2_msgs
nav2_navfn_planner
nav2_planner
nav2_regulated_pure_pursuit_controller
nav2_rotation_shim_controller
nav2_route
nav2_rviz_plugins
nav2_simple_commander
nav2_smac_planner
nav2_smoother
nav2_theta_star_planner
nav2_util
nav2_velocity_smoother
nav2_voxel_grid
nav2_waypoint_follower
nav_2d_msgs
nav_2d_utils
nav_msgs
navigation2
ompl
orocos_kdl_vendor
osrf_pycommon
parameter_traits
pcl_conversions
pcl_msgs
pendulum_control
pendulum_msgs
pluginlib
pybind11_vendor
python_cmake_module
python_orocos_kdl_vendor
python_qt_binding
qt_dotgraph
qt_gui
qt_gui_cpp
qt_gui_py_common
quality_of_service_demo_cpp
quality_of_service_demo_py
rcl
rcl_action
rcl_interfaces
rcl_lifecycle
rcl_logging_interface
rcl_logging_spdlog
rcl_yaml_param_parser
rclcpp
rclcpp_action
rclcpp_components
rclcpp_lifecycle
rclpy
rcpputils
rcutils
realtime_tools
resource_retriever
rmw
rmw_dds_common
rmw_fastrtps_cpp
rmw_fastrtps_shared_cpp
rmw_implementation
rmw_implementation_cmake
robot_state_publisher
ros2_control
ros2_control_test_assets
ros2action
ros2bag
ros2cli
ros2cli_common_extensions
ros2component
ros2controlcli
ros2doctor
ros2interface
ros2launch
ros2lifecycle
ros2multicast
ros2node
ros2param
ros2pkg
ros2plugin
ros2run
ros2service
ros2topic
ros_base
ros_core
ros_environment
ros_workspace
rosbag2
rosbag2_compression
rosbag2_compression_zstd
rosbag2_cpp
rosbag2_interfaces
rosbag2_py
rosbag2_storage
rosbag2_storage_default_plugins
rosbag2_transport
rosgraph_msgs
rosidl_adapter
rosidl_cli
rosidl_cmake
rosidl_default_generators
rosidl_default_runtime
rosidl_generator_c
rosidl_generator_cpp
rosidl_generator_py
rosidl_generator_rs
rosidl_parser
rosidl_runtime_c
rosidl_runtime_cpp
rosidl_runtime_py
rosidl_typesupport_c
rosidl_typesupport_cpp
rosidl_typesupport_fastrtps_c
rosidl_typesupport_fastrtps_cpp
rosidl_typesupport_interface
rosidl_typesupport_introspection_c
rosidl_typesupport_introspection_cpp
rpyutils
rqt_action
rqt_bag
rqt_bag_plugins
rqt_common_plugins
rqt_console
rqt_graph
rqt_gui
rqt_gui_cpp
rqt_gui_py
rqt_image_view
rqt_msg
rqt_plot
rqt_publisher
rqt_py_common
rqt_py_console
rqt_reconfigure
rqt_service_caller
rqt_shell
rqt_srv
rqt_topic
rttest
rviz2
rviz_assimp_vendor
rviz_common
rviz_default_plugins
rviz_ogre_vendor
rviz_rendering
sdl2_vendor
sensor_msgs
sensor_msgs_py
shape_msgs
shared_queues_vendor
slam_toolbox
smclib
spdlog_vendor
sqlite3_vendor
sros2
sros2_cmake
statistics_msgs
std_msgs
std_srvs
stereo_msgs
tango_icons_vendor
tcb_span
teleop_twist_joy
teleop_twist_keyboard
tf2
tf2_bullet
tf2_eigen
tf2_eigen_kdl
tf2_geometry_msgs
tf2_kdl
tf2_msgs
tf2_py
tf2_ros
tf2_ros_py
tf2_sensor_msgs
tf2_tools
tinyxml2_vendor
tinyxml_vendor
tl_expected
tlsf
tlsf_cpp
topic_monitor
tracetools
trajectory_msgs
transmission_interface
tricycle_controller
turtlebot3
turtlebot3_bringup
turtlebot3_cartographer
turtlebot3_description
turtlebot3_example
turtlebot3_fake_node
turtlebot3_gazebo
turtlebot3_manipulation_gazebo
turtlebot3_msgs
turtlebot3_navigation2
turtlebot3_node
turtlebot3_simulations
turtlebot3_teleop
turtlesim
uncrustify_vendor
unique_identifier_msgs
urdf
urdf_parser_plugin
velocity_controllers
visualization_msgs
xacro
yaml_cpp_vendor
zstd_vendor
RS2Team8_Package
ackermann_msgs
action_msgs
action_tutorials_cpp
action_tutorials_interfaces
action_tutorials_py
actionlib_msgs
ament_cmake
ament_cmake_auto
ament_cmake_copyright
ament_cmake_core
ament_cmake_cppcheck
ament_cmake_cpplint
ament_cmake_export_definitions
ament_cmake_export_dependencies
ament_cmake_export_include_directories
ament_cmake_export_interfaces
ament_cmake_export_libraries
ament_cmake_export_link_flags
ament_cmake_export_targets
ament_cmake_flake8
ament_cmake_gen_version_h
ament_cmake_gmock
ament_cmake_gtest
ament_cmake_include_directories
ament_cmake_libraries
ament_cmake_lint_cmake
ament_cmake_pep257
ament_cmake_pytest
ament_cmake_python
ament_cmake_ros
ament_cmake_target_dependencies
ament_cmake_test
ament_cmake_uncrustify
ament_cmake_version
ament_cmake_xmllint
ament_copyright
ament_cppcheck
ament_cpplint
ament_flake8
ament_index_cpp
ament_index_python
ament_lint
ament_lint_auto
ament_lint_cmake
ament_lint_common
ament_package
ament_pep257
ament_uncrustify
ament_xmllint
angles
behaviortree_cpp_v3
bond
bondcpp
builtin_interfaces
camera_calibration_parsers
camera_info_manager
cartographer_ros
cartographer_ros_msgs
class_loader
common_interfaces
composition
composition_interfaces
console_bridge_vendor
control_msgs
control_toolbox
controller_interface
controller_manager
controller_manager_msgs
costmap_queue
cv_bridge
demo_nodes_cpp
demo_nodes_cpp_native
demo_nodes_py
depthimage_to_laserscan
desktop
diagnostic_msgs
diagnostic_updater
diff_drive_controller
domain_coordinator
dummy_map_server
dummy_robot_bringup
dummy_sensors
dwb_core
dwb_critics
dwb_msgs
dwb_plugins
dynamixel_sdk
dynamixel_sdk_custom_interfaces
dynamixel_sdk_examples
effort_controllers
eigen3_cmake_module
example_interfaces
examples_rclcpp_minimal_action_client
examples_rclcpp_minimal_action_server
examples_rclcpp_minimal_client
examples_rclcpp_minimal_composition
examples_rclcpp_minimal_publisher
examples_rclcpp_minimal_service
examples_rclcpp_minimal_subscriber
examples_rclcpp_minimal_timer
examples_rclcpp_multithreaded_executor
examples_rclpy_executors
examples_rclpy_minimal_action_client
examples_rclpy_minimal_action_server
examples_rclpy_minimal_client
examples_rclpy_minimal_publisher
examples_rclpy_minimal_service
examples_rclpy_minimal_subscriber
fastrtps_cmake_module
filters
forward_command_controller
gazebo_dev
gazebo_model_attachment_plugin
gazebo_model_attachment_plugin_msgs
gazebo_msgs
gazebo_no_physics_plugin
gazebo_planar_move_plugin
gazebo_plugins
gazebo_ros
gazebo_ros2_control
gazebo_ros2_control_demos
gazebo_ros_pkgs
gazebo_set_joint_positions_plugin
gazebo_video_monitor_interfaces
gazebo_video_monitor_plugins
gazebo_video_monitor_utils
gazebo_video_monitors
generate_parameter_library
generate_parameter_library_py
geometry2
geometry_msgs
hardware_interface
image_geometry
image_tools
image_transport
imu_sensor_broadcaster
interactive_markers
intra_process_demo
joint_limits
joint_state_broadcaster
joint_trajectory_controller
joy
kdl_parser
keyboard_handler
laser_geometry
launch
launch_ros
launch_testing
launch_testing_ament_cmake
launch_testing_ros
launch_xml
launch_yaml
libcurl_vendor
libstatistics_collector
libyaml_vendor
lifecycle
lifecycle_msgs
logging_demo
map_msgs
message_filters
nav2_amcl
nav2_behavior_tree
nav2_behaviors
nav2_bringup
nav2_bt_navigator
nav2_collision_monitor
nav2_common
nav2_constrained_smoother
nav2_controller
nav2_core
nav2_costmap_2d
nav2_dwb_controller
nav2_lifecycle_manager
nav2_map_server
nav2_mppi_controller
nav2_msgs
nav2_navfn_planner
nav2_planner
nav2_regulated_pure_pursuit_controller
nav2_rotation_shim_controller
nav2_route
nav2_rviz_plugins
nav2_simple_commander
nav2_smac_planner
nav2_smoother
nav2_theta_star_planner
nav2_util
nav2_velocity_smoother
nav2_voxel_grid
nav2_waypoint_follower
nav_2d_msgs
nav_2d_utils
nav_msgs
navigation2
ompl
orocos_kdl_vendor
osrf_pycommon
parameter_traits
pcl_conversions
pcl_msgs
pendulum_control
pendulum_msgs
pluginlib
pybind11_vendor
python_cmake_module
python_orocos_kdl_vendor
python_qt_binding
qt_dotgraph
qt_gui
qt_gui_cpp
qt_gui_py_common
quality_of_service_demo_cpp
quality_of_service_demo_py
rcl
rcl_action
rcl_interfaces
rcl_lifecycle
rcl_logging_interface
rcl_logging_spdlog
rcl_yaml_param_parser
rclcpp
rclcpp_action
rclcpp_components
rclcpp_lifecycle
rclpy
rcpputils
rcutils
realtime_tools
resource_retriever
rmw
rmw_dds_common
rmw_fastrtps_cpp
rmw_fastrtps_shared_cpp
rmw_implementation
rmw_implementation_cmake
robot_state_publisher
ros2_control
ros2_control_test_assets
ros2action
ros2bag
ros2cli
ros2cli_common_extensions
ros2component
ros2controlcli
ros2doctor
ros2interface
ros2launch
ros2lifecycle
ros2multicast
ros2node
ros2param
ros2pkg
ros2plugin
ros2run
ros2service
ros2topic
ros_base
ros_core
ros_environment
ros_workspace
rosbag2
rosbag2_compression
rosbag2_compression_zstd
rosbag2_cpp
rosbag2_interfaces
rosbag2_py
rosbag2_storage
rosbag2_storage_default_plugins
rosbag2_transport
rosgraph_msgs
rosidl_adapter
rosidl_cli
rosidl_cmake
rosidl_default_generators
rosidl_default_runtime
rosidl_generator_c
rosidl_generator_cpp
rosidl_generator_py
rosidl_generator_rs
rosidl_parser
rosidl_runtime_c
rosidl_runtime_cpp
rosidl_runtime_py
rosidl_typesupport_c
rosidl_typesupport_cpp
rosidl_typesupport_fastrtps_c
rosidl_typesupport_fastrtps_cpp
rosidl_typesupport_interface
rosidl_typesupport_introspection_c
rosidl_typesupport_introspection_cpp
rpyutils
rqt_action
rqt_bag
rqt_bag_plugins
rqt_common_plugins
rqt_console
rqt_graph
rqt_gui
rqt_gui_cpp
rqt_gui_py
rqt_image_view
rqt_msg
rqt_plot
rqt_publisher
rqt_py_common
rqt_py_console
rqt_reconfigure
rqt_service_caller
rqt_shell
rqt_srv
rqt_topic
rttest
rviz2
rviz_assimp_vendor
rviz_common
rviz_default_plugins
rviz_ogre_vendor
rviz_rendering
sdl2_vendor
sensor_msgs
sensor_msgs_py
shape_msgs
shared_queues_vendor
slam_toolbox
smclib
spdlog_vendor
sqlite3_vendor
sros2
sros2_cmake
statistics_msgs
std_msgs
std_srvs
stereo_msgs
tango_icons_vendor
tcb_span
teleop_twist_joy
teleop_twist_keyboard
tf2
tf2_bullet
tf2_eigen
tf2_eigen_kdl
tf2_geometry_msgs
tf2_kdl
tf2_msgs
tf2_py
tf2_ros
tf2_ros_py
tf2_sensor_msgs
tf2_tools
tinyxml2_vendor
tinyxml_vendor
tl_expected
tlsf
tlsf_cpp
topic_monitor
tracetools
trajectory_msgs
transmission_interface
tricycle_controller
turtlebot3
turtlebot3_bringup
turtlebot3_cartographer
turtlebot3_description
turtlebot3_example
turtlebot3_fake_node
turtlebot3_gazebo
turtlebot3_manipulation_gazebo
turtlebot3_msgs
turtlebot3_navigation2
turtlebot3_node
turtlebot3_simulations
turtlebot3_teleop
turtlesim
uncrustify_vendor
unique_identifier_msgs
urdf
urdf_parser_plugin
velocity_controllers
visualization_msgs
xacro
yaml_cpp_vendor
zstd_vendor