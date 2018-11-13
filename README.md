RAIL Velodyne ~ under construction!!!
========

This rail_velodyne repository includes custom packages required to install, simulate, and use the Velodyne VLP-16 with the Fetch. The drivers which bring up the physical VLP-16 were forked from [here](https://github.com/ros-drivers/velodyne) while the customized launch files to start the Fetch simluations that include the VLP-16 reuse models from [here](https://bitbucket.org/DataspeedInc/velodyne_simulator).

# Dependencies
Fetch packages for simulation. Please see [their site](http://docs.fetchrobotics.com/index.html) for installation instructions.

# Install the simulation launch files and drivers to a PC

1. Begin by installing the Gazebo simulation packages for the VLP-16: `sudo apt-get install ros-indigo-velodyne-simulator`
2. Next clone this repo into a catkin workspace you intend to use.
3. Install dependencies via rosdep: `rosdep install --from-paths src --ignore-src --rosdistro indigo -y`
3. Run `catkin_make` in the catkin workspace to build the pacakges and remember to source the `setup.bash` in the `devel` folder.

# Configure Fetch Networking for VLP-16

1. Power the VLP-16 and plug it into Fetch's external Ethernet.
2. Access Fetch via ssh or some other method and modify the `/etc/network/interfaces` file to include the following regarding `eth0` (this will allow the Velodyne to be accessed):
```
auto eth0
iface eth0 inet static
    address 192.168.1.11
    netmask 255.255.255.0
    gateway 192.168.1.1
```
3. Restart Fetch
4. Verify network configuration by running `ifconfig`. The network setting for `eth0` should read similar to `inet addr:192.168.1.11  Bcast:192.168.1.255  Mask:255.255.255.0`

# Verify Installations
## Verify Installation of simulator to PC
