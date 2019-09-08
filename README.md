# MAV navigation in an enclosed environment

MAV models from [rotorS](https://github.com/ethz-asl/rotors_simulator).

# System Requirements

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
wget https://raw.githubusercontent.com/tidota/mav-tunnel-nav/master/mav_tunnel_nav.rosinstall
wstool init . mav_tunnel_nav.rosinstall
wstool update
rosdep install --from-paths . --ignore-src --rosdistro=melodic --skip-keys "octomap_ros" -y
sudo apt install ros-melodic-octomap-ros
sudo apt install ros-melodic-rotors-*
```

## libmav_msgs.so
There is a problem related to libmav_msgs.so. The library file is not included if rotorS was installed by apt.
https://github.com/ethz-asl/rotors_simulator/pull/506

download [libmav_msgs.so](https://drive.google.com/open?id=1VtutTPUiwGTCpCeF32B7m4G2noUGp75N) and copy it to

```
/opt/ros/melodic/lib/
```

# Setup of the Simulation Environments

To run a simulation, you need world models.

Download `models.zip` from https://drive.google.com/file/d/1XFQKM-PIM0M39C8rlT6tAcLINDL0DOhb/view?usp=sharing

Then, extend it and move the items into `~/.gazebo/models/`.
