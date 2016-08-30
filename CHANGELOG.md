Change log
==========

1.2.0 (2016-08-30)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * cisst_msgs: added StringStamped
  * cisst_ros_bridge: added more data conversion methods:
    * Joy from event button pressed
    * Float64MultiArray from dynamic matrix
    * PointCloud from std::vector<vct3>
  * cisst_ros_bridge: use cisstMultiTask timestamp to set ROS time for messages with header
* Bug fixes:
  * gcc: minor fixes to avoid warnings


1.1.0 (2016-01-08)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * cisst_ros_bridge: Added more data conversion methods (BoolStamped)
* Bug fixes:
  * None


1.0.1 (2015-10-18)
==================

* Change log file created
* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * cisst_ros_bridge: Added prmStateJoint data type
  * cisst_ros_bridge: Added support to convert events with std::string to ROS log messages
  * cisst_ros_bridge: Added more data conversion methods (int32, mtsFrm4x4)
* Bug fixes:
  * cisst_ros_bridge: Only read/convert if there are active subscribers
