# RAIL Velodyne ~ under construction!!!

This rail_velodyne repository includes custom packages required to install, simulate, and use the Velodyne VLP-16 with the Fetch. The drivers which bring up the physical VLP-16 were forked from [here](https://github.com/ros-drivers/velodyne) while the customized launch files to start the Fetch simluations that include the VLP-16 reuse models from [here](https://bitbucket.org/DataspeedInc/velodyne_simulator).

# Dependencies
Fetch packages for simulation. Please see [their site](http://docs.fetchrobotics.com/index.html) for installation instructions.

## Fetch / Velodyne with Simulation
This goes through the process of installing and running the simulations for Fetch including the VLP-16.

### Install Fetch / Velodyne Simulator to a PC
1. Begin by installing the Gazebo simulation packages for the VLP-16: `sudo apt-get install ros-indigo-velodyne-simulator`
2. Next clone this repo into a catkin workspace you intend to use.
3. Install dependencies via rosdep: `rosdep install --from-paths src --ignore-src --rosdistro indigo -y`
4. Run `catkin_make` in the catkin workspace to build the pacakges and remember to source the `setup.bash` in the `devel` folder.

### Verify Fetch / Velodyne Simulator Installation
1. Run the simulator via `roslaunch fetch_velodyne playground.launch`. This should begin Gazebo with the Fetch including the VLP-16 on it's pan joint in the defualt playground environment.
2. Check the topics being published via `rostopic list`. There should be a topic called `/velodyne/depth/points`.
3. Visualize the points being published in rviz via `rosrun rviz rviz -f velodyne` using the PointCloud2 rviz pluggin.

## Fetch / Velodyne with physical hardware
This goes through the installation of the VLP-16 and it's drivers into the Fetch. NOTE: To simply use the VLP-16 once it has already been installed (most cases), skip to **Toggling the Velodyne on Fetch** below.

### Configure Fetch Networking for VLP-16
1. Power the VLP-16 and plug it into Fetch's external Ethernet.
2. Access Fetch via ssh or some other method and modify the `/etc/network/interfaces` file to include the following regarding `eth0` (this will allow the Velodyne to be accessed):
```
auto eth0
iface eth0 inet static
    address 192.168.1.11
    netmask 255.255.255.0
    gateway 192.168.1.1
```
3. Restart Fetch to reset the networking service.

### Verify Fetch Networking Configuration for VLP-16
1. Verify network configuration by running `ifconfig`. The network setting for `eth0` should read similar to `inet addr:192.168.1.11  Bcast:192.168.1.255  Mask:255.255.255.0`
2. Access Fetch via ssh or some other method and open a browser to access the web-server hosted by the VLP-16 at `192.168.1.201`. If networking was set up properly, you should be able to both ping this IP and view the web-server default page *from a web-browser on the Fetch*.

### Install VLP-16 drivers to the Fetch
1. Access Fetch via ssh or some other method and clone this repo into a catkin workspace you intend to use.
2. Access Fetch via ssh or some other method and install dependencies via rosdep for the workspace: `rosdep install --from-paths src --ignore-src --rosdistro indigo -y`
3. Access Fetch via ssh or some other method and run `catkin_make` in the catkin workspace to build the pacakges and remember to source the `setup.bash` in the `devel` folder.

### Verify VLP-16 drivers Installation to the Fetch
1. Access Fetch via ssh or some other method and run `roslaunch velodyne_pointcloud VLP16_points.launch` to start the drivers.
2. Check the topics being published via `rostopic list`. There should be a topic called `/velodyne/depth/points`.
3. Visualize the points being published in rviz via `rosrun rviz rviz -f velodyne` using the PointCloud2 rviz pluggin.

### Configure the Fetch URDF and Startup Services to include the VLP-16
1. This section will be involved, but once properly conifgured the VLP-16 can be toggled by commenting a single line. Begin by locating the startup service that launches all the drivers for the robot. From the Fetch [documentation](http://docs.fetchrobotics.com/computer.html), it should located at `/etc/init/robot.conf`. We will edit this file, but before doing so, save a copy `sudo cp /etc/init/robot.conf /etc/init/robot.conf.bak`.
2. Open the file for editting in vi via `sudo vi /etc/init/robot.conf` Notice this file sources the default ROS `setup.bash` then launches the robot at line 9. Copy this line directly below and modify it to *also* source the workspace where the VLP-16 drivers have been installed as well as launch the robot via a new file we will create called `robot_velodyne.launch`. This should look something like:
```
exec su ros -c ". /opt/ros/indigo/setup.bash && . /PATH_TO_VLP16_DRIVERS/devel/setup.bash && roslaunch /etc/ros/indigo/robot_velodyne.launch --wait"
```
3. Now comment the original line so that instead the line you just wrote gets called. Write the changes and quit vim.
4. 
# Verify Installations
## Verify Installation of simulator to PC
