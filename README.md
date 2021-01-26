# cisst-ros

cisst/ROS integration package.  This package contains components that can be used to provide a bridge between [cisstMultiTask components](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts) and ROS 1.  These components can be used to map a cisstMultiTask command or event to ROS topics and services.

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * ROS 1 (tested with Kinetic, Melodic and Noetic)
 * CRTK ROS messages: https://github.com/collaborative-robotics/crtk_msgs

# Build

You can find some documentation re. compiling cisst and SAW components in the [dVRK wiki](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild#catkin-build-and-rosinstall).  All packages can be cloned in your catkin workspace and built using `catkin build` (Python Catkin Build Tools).  `catkin make` is not supported.

# Main packages

The main three ROS packages in this repository are:
* `cisst_msgs`: a collection of messages matching `cisst` data types with no obvious equivalent in ROS
* `cisst_ros_bridge`: base class to bridge between cisst and ROS (`mtsROSBridge`).  This library also includes methods to convert bach and forth between cisst and ROS data types
* `cisst_ros_crtk`: component using multiple instances of `mtsROSBridge` to automatically bridge a [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API) compatible cisst interface to ROS

# Examples

The cisst-ros package for the following applications to create ROS interfaces:
* dVRK: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki (da Vinci Research Kit)
* sawNDITracker: https://github.com/jhu-saw/sawNDITracker (NDI serial port based optical and magnetic trackers)
* sawForceDimensionSDK: https://github.com/jhu-saw/sawForceDimensionSDK (Force Dimension haptic devices and Novint Falcon)
* ... and many more cisst/SAW components found in https://github.com/jhu-saw

