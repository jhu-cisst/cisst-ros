# About
Cisst ROS integration packages 

# Dependencies:
 * cisst libaray 
 
# Install

```sh
# clone cisst-ros 
$ cd /PATH/TO/CATKIN_WS/src  
$ git clone git@github.com:jhu-cisst/cisst-ros.git

# catkin make 
$ cd ..
$ catkin_make    # you may need to specify cisst build path using cmake 
                 # default cisst\_DIR path: PATH/TO/CISST/build/cisst/

```

# Package Description
* cisst_msgs: ros messages for cisst data type
* cisst_ros_bridge:
   * bridge component for cisst to ros
   * data type conversion methods 

# Todo 
* rename mtsROSTOCISST is necessary

