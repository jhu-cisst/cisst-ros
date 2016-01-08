# About

cisst/ROS integration packages 

# Dependencies:
 * cisst libraries: see https://github.com/jhu-cisst/cisst/wiki to download and install the cisst libraries and SAW components
 * ROS: you can check if ROS is installed by checking the content of `/opt/ros`.  If ROS is not installed, follow instructions from http://www.ros.org.

# Initialize your catkin ROS workspace

As of October 2015, we strongly recommend to install cisst/SAW using the ROS catkin build tools:
https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros

Make sure ROS is in your path.  You can also "source" the setup script that comes with your ROS distribution (example for ROS hydro):
```sh
source /opt/ros/hydro/setup.bash
```

Create the main directories for ROS and initialize the workspace:
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin init
```

First build to initialize the environment:
```sh
cd ~/catkin_ws/
catkin build
```

To setup your path and find the ROS commands easily:
```sh
source ~/catkin_ws/devel/setup.bash
```

# Install cisst-ROS

This part is not needed if you installed _cisst/SAW_ using the catkin build tools.   _cisst-ros_ comes with _cisst/SAW_.

Get the sources:
```sh
cd ~/catkin_ws/src  
git clone http://github.com/jhu-cisst/cisst-ros.git
```

Build:
```sh 
cd ~/catkin_ws
catkin build   # you may need to specify cisst build path using cmake 
               # default cisst\_DIR path: PATH/TO/CISST/build/cisst/
```
**NOTE**   
If you have a catkin_make error complaining about not finding cisst, please 
specify cisst path manually. 
```sh
# cd to build dir
cd ~/catkin_ws/build
# start ccmake 
ccmake .
```
In CMake, set `cisst_DIR` to PATH/TO/CISST/build/cisst/

# Package Description for cisst/ROS
* cisst_msgs: ros messages for cisst data types
* cisst_ros_bridge:
   * bridge component for cisst to ros
   * data type conversion methods 
