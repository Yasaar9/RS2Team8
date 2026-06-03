# RS2 Team 8 : Social Tour Guide Robot

Autonomous social tour guide robot for a gallery environment, built on ROS 2 Humble with a TurtleBot3 Waffle Pi. The robot navigates to gallery artefacts and facilities on request, provides spoken descriptions, detects and avoids obstacles, and recovers autonomously when blocked.

**Team:** Mohammad Baboonboy, Htet Phone Aung (Peter), Jerry Sun, Liam Farabi  
**Subject:** 41069 Robotics Studio 2 : University of Technology Sydney

---

## Repository Structure

```
~/git/RS2Team8/
├── README.md
├── .gitignore
└── r2_dTour/
    ├── 0_maps/                        ← Map files and waypoint configs
    │   ├── simulation_map.yaml/.pgm   ← Gazebo world map
    │   ├── gallery_map.yaml/.pgm      ← Real gallery map
    │   ├── simulation_waypoints.txt   ← Simulation waypoints
    │   └── gallery_waypoints.txt      ← Real gallery waypoints
    ├── 0_models/                      ← Custom Gazebo models
    ├── 0_obstacles/                   ← Timestamped obstacle photos (auto-generated)
    ├── DynamixelSDK/                  ← ROBOTIS SDK
    ├── turtlebot3_msgs/               ← ROBOTIS messages
    ├── turtlebot3/                    ← ROBOTIS TurtleBot3 package
    ├── turtlebot3_simulations/        ← ROBOTIS simulation package
    └── rs2_team8/                     ← Custom package
        ├── package.xml
        ├── setup.py
        ├── launch/
        │   └── rs2_tour.launch.py     ← Master launch file
        ├── config/
        │   ├── params/
        │   │   └── nav2_params.yaml   ← Nav2 configuration
        │   ├── maps/
        │   └── rviz/
        └── rs2_team8/
            ├── __init__.py
            └── nodes/
                ├── navigation.py      ← Motion planning and control
                ├── ui.py              ← Touchscreen GUI and speech
                ├── send_waypoint.py   ← CLI testing utility
                └── detector.py        ← Perception and mapping (run separately)

~/turtlebot3_ws/                       ← Colcon build workspace (not in git)
└── src/
    └── r2_dTour -> ~/git/RS2Team8/r2_dTour   ← symlink
```

---

## Installation

### 1. Install ROS 2 Humble

Follow the official Debian install guide:  
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### 2. Install dependencies

```bash
# TurtleBot3 and Nav2
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# SLAM (cartographer for simulation, slam_toolbox for real robot)
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-slam-toolbox

# Camera
sudo apt install ros-humble-image-*

# Audio (text-to-speech)
sudo apt install python3-pip
sudo apt install mpg123
pip3 install gtts

# GUI
sudo apt install python3-tk

# AI responses (UI falls back if absent)
pip3 install ollama
sudo snap install ollama
```

TurtleBot3 e-Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

### 3. Clone the repository

```bash
mkdir -p ~/git
cd ~/git
git clone git@github.com:Yasaar9/RS2Team8.git
```

### 4. Create the workspace and symlink

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
ln -s ~/git/RS2Team8/r2_dTour
```

### 5. Build and configure

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install --cmake-args -Wno-dev

# Add to ~/.bashrc (run once)
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=71' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
source ~/.bashrc
```

### 6. Rebuild from scratch (if needed)

```bash
cd ~/turtlebot3_ws
rm -rf build install log
colcon build --symlink-install --cmake-args -Wno-dev
source ~/.bashrc
```

### 7. VS Code installation note (RViz conflict)

Do **not** install VS Code via snap — the `core20` snap package injects libraries that conflict with RViz and cause a `symbol lookup error: libpthread` crash. Install VS Code via apt instead:

```bash
# Remove snap version if already installed
sudo snap remove code

# Install via apt
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code
```

Open a fresh terminal after this and RViz will work normally.

---

## Running the System

> `map`, `waypoints_file`, and `params_file` are **required**. The launch will fail immediately with a clear error if any are missing or point to a non-existent file.

### Simulation

```bash
ros2 launch rs2_team8 rs2_tour.launch.py use_sim:=true \
  map:=$HOME/git/RS2Team8/r2_dTour/0_maps/simulation_map.yaml \
  waypoints_file:=$HOME/git/RS2Team8/r2_dTour/0_maps/simulation_waypoints.txt \
  params_file:=$HOME/git/RS2Team8/r2_dTour/rs2_team8/config/params/nav2_params.yaml
```

Gazebo opens → Nav2 activates after 4 s → navigation_node and UI start after 10–11 s. Initial pose is set automatically at (0, 0).

### Real robot

SSH into robot on two terminals
 
```bash
ssh ubuntu@192.168.0.XXX
```

Set time on the robot (replace with your laptop's IP)

```bash
sudo date -s "$(ssh youruser@192.168.0.YYY 'date -u')" # sudo date -s "$(ssh jsunne@192.168.0.239 'date -u')"
```

launch bringup

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
On second SSH terminal on the robot (via SSH):

```bash
ros2 run camera_ros camera_node --ros-args -p format:=RGB888
```
Run all nodes

```bash
ros2 launch rs2_team8 rs2_tour.launch.py use_sim:=false \
  map:=$HOME/git/RS2Team8/r2_dTour/0_maps/gallery_map.yaml \
  waypoints_file:=$HOME/git/RS2Team8/r2_dTour/0_maps/gallery_waypoints.txt \
  params_file:=$HOME/git/RS2Team8/r2_dTour/rs2_team8/config/params/nav2_params.yaml
```

Nav2 and RViz open after 2 s → navigation_node and UI start after 15–16 s.

**Before pressing any UI buttons:**
1. In RViz click **2D Pose Estimate** and drag the green arrow to the robot's approximate position and heading on the map
2. Once the particle cloud tightens in RViz, the system is ready

---

## Running Each Node Separately

> Source your workspace first in every terminal:
> ```bash
> source ~/.bashrc
> ```


#### 1. Gazebo world
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### 2. Nav2
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=$HOME/git/RS2Team8/r2_dTour/0_maps/simulation_map.yaml \
  params_file:=$HOME/git/RS2Team8/r2_dTour/rs2_team8/config/params/nav2_params.yaml \
  use_rviz:=false
```

#### 3. Navigation node
```bash
ros2 run rs2_team8 navigation_node \
  --ros-args \
  -p use_sim:=true \
  -p waypoints_file:=$HOME/git/RS2Team8/r2_dTour/0_maps/simulation_waypoints.txt
```

#### 4. UI node
```bash
ros2 run rs2_team8 ui_node
```

---

## Test Waypoint Navigation (CLI)

Send goals without the UI using the `send_waypoint` utility:

```bash
ros2 run rs2_team8 send_waypoint artifact_1
ros2 run rs2_team8 send_waypoint artifact_2
ros2 run rs2_team8 send_waypoint artifact_3
ros2 run rs2_team8 send_waypoint artifact_4
ros2 run rs2_team8 send_waypoint toilet_1
ros2 run rs2_team8 send_waypoint fire_exit_1
ros2 run rs2_team8 send_waypoint fire_exit_2
ros2 run rs2_team8 send_waypoint home
ros2 run rs2_team8 send_waypoint cancel

# Send a raw coordinate (x y yaw_degrees)
ros2 run rs2_team8 send_waypoint pose 2.5 -1.0 90
```

Monitor navigation status:
```bash
ros2 topic echo /navigation/status
```

---

## Connecting to the Real TurtleBot3

### Network setup

Connect to the TurtleBot3 Wi-Fi network. Ensure your laptop's `~/.bashrc` has the correct domain ID to match the robot:

```bash
export ROS_DOMAIN_ID=71
export TURTLEBOT3_MODEL=waffle_pi
```

> **Note:** If you have `ROS_DOMAIN_ID=67` or any other value in your `.bashrc`, change it to `71` — a mismatch means your laptop and robot cannot see each other's topics. Verify with `ros2 topic list` after connecting; you should see `/scan`, `/odom`, `/map` etc.

### SSH into the robot

```bash
ssh ubuntu@192.168.0.XXX
# Password: turtlebot
```

The robot's `~/.bashrc` should already contain (set domain ID to 71):

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/turtlebot3_ws/install/setup.bash
export LDS_MODEL=LDS-01
export TURTLEBOT3_MODEL=waffle_pi
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=waffle
export ROS_DOMAIN_ID=71
```
To check:

```bash
nano ~/.bashrc
```
### Start the robot bringup (on the robot via SSH)

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Leave this terminal running. All following commands are on your **laptop**.

### Teleop

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

> **Note:** The teleop terminal must be the **actively focused window** for WASD keys to register. Clicking another window (e.g. RViz) will cause keys to stop working. Spacebar sends a stop command to the robot directly and will still work regardless — if spacebar works but WASD does not, click the teleop terminal to refocus it.

### Camera

Install the dependency locally:

```bash
sudo apt install ros-humble-image-*
```

On the robot (via SSH):
```bash
ros2 run camera_ros camera_node --ros-args -p format:=RGB888
```

On your local machine:
```bash
ros2 run rqt_image_view rqt_image_view
```
Then choose topic: /camera/image_raw/compressed

---

## Building the Gallery Map

 
> **Note:** Cartographer is the official ROBOTIS SLAM method for TurtleBot3.
 
Open **four terminals** on your laptop. Source `~/.bashrc` in each before running any command.
 
### Terminal 1 — Robot bringup (SSH into robot)
 
```bash
ssh ubuntu@192.168.0.XXX
ros2 launch turtlebot3_bringup robot.launch.py
```
 
Leave this running for the entire mapping session.
 
### Terminal 2 — Cartographer
 
```bash
source ~/.bashrc
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=false
```
 
If RViz does not show the map, check the following in the RViz window:
 
- Set **Fixed Frame** (top left under Global Options) to `map`
- Click **Add** → **By Topic** → add `/map` if not already present
- Click **Add** → **By Topic** → add `/scan` if not already present
> **Verify the robot is visible** by checking `ros2 topic list` in a separate terminal — you should see `/scan`, `/odom`, `/map` etc. If the list is empty or missing these topics, your `ROS_DOMAIN_ID` does not match the robot (must be `71`).
 
### Terminal 3 — Teleop
 
```bash
source ~/.bashrc
ros2 run turtlebot3_teleop teleop_keyboard
```
 
Keep the teleop terminal focused (clicked) while driving. Drive slowly. Avoid driving in a straight line and rotating simultaneously. Cartographer is sensitive to fast or combined linear+rotational motion. 
 
### Save the map
 
Once the map looks complete in RViz, open a **fourth terminal** and run:
 
```bash
source ~/.bashrc
ros2 run nav2_map_server map_saver_cli -f \
  $HOME/git/RS2Team8/r2_dTour/0_maps/gallery_map
```

---

## Manually Recording the Waypoints

### Terminal 1 — Robot bringup (SSH into robot)
 
```bash
ssh ubuntu@192.168.0.XXX
ros2 launch turtlebot3_bringup robot.launch.py
```
 
Leave this running for the entire mapping session.
 
### Terminal 2 — Nav2 with the new map
 
```bash
source ~/.bashrc
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=$HOME/git/RS2Team8/r2_dTour/0_maps/gallery_map.yaml \
  params_file:=$HOME/git/RS2Team8/r2_dTour/rs2_team8/config/params/nav2_params.yaml \
  use_rviz:=true
```
 
When RViz opens, click "2D Pose Estimate" and place the arrow at the robot's starting position. 
 
### Terminal 3 — Teleop
 
```bash
source ~/.bashrc
ros2 run turtlebot3_teleop teleop_keyboard
```
 
Keep the teleop terminal focused (clicked) while driving. Drive slowly. Avoid driving in a straight line and rotating simultaneously. Cartographer is sensitive to fast or combined linear+rotational motion. 
 
### Record coordinates
 
Drive the robot to each artefact/facility position using teleop, then in a new terminal record the AMCL pose:
 
```bash
source ~/.bashrc
ros2 topic echo /amcl_pose --once
```

---

## Useful Commands

```bash
# List active ROS nodes
ros2 node list

# List active topics
ros2 topic list

# Check your custom package is built
ros2 pkg list | grep rs2_team8

# Verify navigation status
ros2 topic echo /navigation/status

# Verify AMCL is localising
ros2 topic echo /amcl_pose --once
```
---