# Obstacle avoider using turtle bot
This project involves the development of an basic obstacle avoidance algorithm for turtlebot3 robot. 

# Dependencies and Assumptions:

* System configuration: Ubuntu 22.04
* ROS2 Humble should be installed, if not you can follow the instructions below
* ament_cmake
* rclcpp
* std_msgs
  
# Setting up the dependencies, skip to next step if ROS2 Humble and Clangd already installed 
## Installing ROS2 Humble for Ubuntu 22.04 (http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 
You should follow the link for more updated instructions

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
```

Source the setup file to start ROS2, add the following line in the .bashrc file of your system:
```bash
source /opt/ros/humble/setup.bash
```

## Install Clang

```bash
sudo apt install clang
export CC=clang
export CXX=clang++
colcon build --cmake-force-configure
```
## Install turtlebot3:

```bash
sudo apt install ros-humble-turtlebot3*
```

## Install Gazebo

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Clone the repo in to the src folder of your ROS Workspace:
```
cd ~/ros2_ws/src
git clone https://github.com/sanchit2843/ObstacleAvoider
```

## Build your package
```bash
colcon build --packages-select ObstacleAvoider
```
## Source setup files
```bash
. install/setup.bash
```
## To launch obstacle avoiding algorithm on turtlebot3:
```bash
ros2 launch ObstacleAvoider obstacle_avoider_launch.py
```

## To launch walker algorithm and record all topics using rosbag:

```bash
ros2 launch ObstacleAvoider obstacle_avoider_launch.py record_topics:=True
```
## To check the content of ROS bag:

```bash
ros2 bag info tb3_walker_bag/
```

## Static code analysis
### Cpplint
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp &> ../results/cpplint.txt
```
### Cppcheck
```
cppcheck --enable=all --std=c++17 src/*.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ../results/cppcheck.txt
```
