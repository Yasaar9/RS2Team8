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
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc


## Rebuild fresh

cd ~/turtlebot3_wsx
rm -rf build install log
colcon build --symlink-install
source ~/.bashrc

## Run Simulation

### Burger model
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

### Waffle model
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

### Waffle Pi model
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py


FOLDER STRUCTURE
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