RAIL Velodyne ~ under construction!!!
========

This rail_velodyne repository includes custom packages required to install, simulate, and use the Velodyne VLP-16 with the Fetch. The drivers which bring up the physical VLP-16 were forked from [here](https://github.com/ros-drivers/velodyne) while the customized launch files to start the Fetch simluations that include the VLP-16 reuse models from [here](https://bitbucket.org/DataspeedInc/velodyne_simulator).

# Dependencies
Fetch packages for simulation. Please see [their site](http://docs.fetchrobotics.com/index.html) for installation instructions.

# Install the package

1. Begin by installing the Gazebo simulation packages for the VLP-16: `sudo apt-get install ros-indigo-velodyne-simulator`
2. Next clone this repo into a catkin workspace you intend to use.
3. Run `catkin_make` in the catkin workspace to build the pacakges and remember to source the `setup.bash` in the `devel` folder.

# Install the VLP-16 to Fetch



# Verify Install
