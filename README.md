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

Make a new catkin workspace.

Assuming you are in the workspace, make a `src` directory in it.
```
mkdir src
cd src
```

Then, run the following commands.
```
wget https://raw.githubusercontent.com/tidota/iris-tunnel-nav/master/iris_tunnel_nav.rosinstall
wstool init . iris_tunnel_nav.rosinstall
wstool update
rosdep install --from-paths src --ignore-src --rosdistro=melodic --skip-keys "octomap_ros" -y
sudo apt install ros-melodic-octomap-ros
sudo apt install ros-melodic-rotors-*
```

# Setup

To run a simulation, you need world models.
The following commands download the files and install them into `~/.gazebo/models`.

```
TODO
```