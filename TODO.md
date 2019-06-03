# TODO list

- [x] Migrate package.xml and CMakeLists.txt from the old repo
- [x] Modify the files
- [x] Installation instructions in README.md

  seems like the rotorS packages are available as Debian packages.

  `sudo apt install ros-melodic-rotors-*`

  maybe, I can just add `rotors-simulator` in package.xml? => no it didn't..

- [x] Make a script to download and setup a world model in .gazebo

  Google Drive now does not let command-line based downloading...?
  OK, just provide instruction to install them.

- [x] Make and test a launch file
- [x] Rename the package "mav_tunnel_nav"
- [x] Rename the github repo as well
- [x] Solve the problem of library path (rotors_gazebo_plugins are not loaded?)

  - the rotors package itself has a problem to load plugins?

    https://github.com/ethz-asl/rotors_simulator/pull/506
    seems like libmav_msgs.so is missing

    The error messages disappeared after copying `libmav_msgs` to `/opt/ros/melodic/lib`.

- [x] Setup joy control by Sanwa gamepad.

  - the world plugin `librotors_gazebo_ros_interface_plugin.so` is necessary in a world file.

- [ ] Try to use only IMU for position estimation

- [ ] Update the installation+setup instructions
- [ ] Add a manual control(?)
