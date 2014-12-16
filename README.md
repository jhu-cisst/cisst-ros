# About

cisst/ROS integration packages 

# Dependencies:
 * cisst libraries: see https://github.com/jhu-cisst/cisst/wiki to download and install the cisst libraries and SAW components
 * ROS: you can check if ROS is installed by checking the content of `/opt/ros`.  If ROS is not installed, follow instructions from http://www.ros.org.

# Initialize your catkin ROS workspace

Create the main directories for ROS and initialize the workspace:
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

First build to initialize the environment:
```sh
cd ~/catkin_ws/
catkin_make
```

To setup your path and find the ROS commands easily:
```sh
source devel/setup.bash
```

# Install

Get the sources:
```sh
cd ~/catkin_ws/src  
git clone http://github.com/jhu-cisst/cisst-ros.git
```

Build:
```sh 
cd ~/catkin_ws
catkin_make    # you may need to specify cisst build path using cmake 
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

# Package Description
* cisst_msgs: ros messages for cisst data types
* cisst_ros_bridge:
   * bridge component for cisst to ros
   * data type conversion methods 
