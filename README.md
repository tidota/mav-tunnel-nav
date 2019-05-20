# Iris MAV navigation in an enclosed environment

Iris model from [rotorS](https://github.com/ethz-asl/rotors_simulator).

# Environment

- Ubuntu 18.04 Bionic
- ROS Melodic
- Gazebo9

# Installation

If you do not have wstool,
```
sudo apt-get install python-wstool
```

Assume you are in the catkin workspace.

Make a `src` directory if you do not have it yet.
```
mkdir src
cd src
```
Then, run the following commands.
```
wget https://raw.githubusercontent.com/tidota/iris-tunnel-nav/master/iris_tunnel_nav.rosinstall
wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
wstool init . iris_tunnel_nav.rosinstall
wstool merge rotors_hil.rosinstall
wstool update
rosdep install --from-paths src --ignore-src --rosdistro=melodic --skip-keys "octomap_ros" -y
sudo apt install ros-melodic-octomap-ros
```