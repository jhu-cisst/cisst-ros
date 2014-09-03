# About
cisst ROS integration packages 

# Dependencies:
 * cisst libraries and ROS 
 
# Install

```sh
# clone cisst-ros 
$ cd /PATH/TO/CATKIN_WS/src  
$ git clone http://github.com/jhu-cisst/cisst-ros.git

# catkin make 
$ cd ..
$ catkin_make    # you may need to specify cisst build path using cmake 
                 # default cisst\_DIR path: PATH/TO/CISST/build/cisst/

```
**NOTE**   
If you have a catkin_make error complaining about not finding cisst, please 
specify cisst path manually. 

```sh
# cd to build dir
cd to /path/to/ros_workspace/build

# start ccmake 
ccmake .

# set cisst_DIR, if you are dvrk user, 
# path should be ~/dev/cisst/build/cisst
cisst_DIR  PATH/TO/CISST/build/cisst/
```


# Package Description
* cisst_msgs: ros messages for cisst data types
* cisst_ros_bridge:
   * bridge component for cisst to ros
   * data type conversion methods 
