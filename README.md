<!--ts-->
   * [cisst-ros](#cisst-ros)
   * [Links](#links)
   * [Dependencies](#dependencies)
   * [Build](#build)
   * [Main packages](#main-packages)
      * [cisst ROS bridge](#cisst-ros-bridge)
      * [cisst ROS CRTK bridge](#cisst-ros-crtk-bridge)
   * [Examples](#examples)

<!-- Added by: anton, at: 2021-01-28T15:59-05:00 -->

<!--te-->

# cisst-ros

cisst/ROS integration package.  This package contains components that can be used to provide a bridge between  and ROS 1.  These components can be used to map a cisstMultiTask command or event to ROS topics and services.

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
* `cisst_ros_bridge`: base class to bridge between cisst and ROS (`mtsROSBridge`).  This library also includes methods to convert back and forth between cisst and ROS data types
* `cisst_ros_crtk`: component using multiple instances of `mtsROSBridge` to automatically bridge a [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API) compatible cisst interface to ROS

## cisst ROS bridge

The overall idea is to create a bridge component between the *cisstMultiTask* component for your device (e.g. robot) and ROS.  The bridge will have ROS publishers to publish data that comes from the device's component and ROS subscribers to send commands to the device's component.

This package doesn't contain any executable nor components that can be used as-is (e.g. through dynamic loading).  The base class `mtsROSBridge` has to be "populated" to bridge the cisstMultiTask commands and events to ROS topics or services.  The simplest solution is to create an instance of `mtsROSBridge` and then call one of the `mtsROSBridge::AddXYZ` methods to add topics or services.  It is also possible to create a new class derived from `mtsROSBridge` and add topics or services in the constructor of the derived class.

For example, to publish to a ROS topic using a cisst read command, one would use:
```c++
bridge->AddPublisherFromCommandRead<std::string, std_msgs::String>
    ("Robot", "Status", "/my_device/status");
```
This call will add the required interface "Robot" to the bridge if it doesn't already exist.  The read function "Status" will be added to the required interface "Robot" (see [cisstMultiTask components](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts) to understand what *required interface*, *command* and *function* are).  Note that this method is templated.  The first template parameter is the type of object used by the *cisst* "device".  The second parameter is the ROS message type that will be published.

As for all *cisstMultiTask* components, the bridge now needs to be added to the component manager and connected to the provided interface from your device.  The provided interface name doesn't have to match the required interface's name.  So assuming the "device" has a provided interface called "Main", the code would look like:
```c++
manager->AddComponent(bridge);
manager->Connect(bridge->GetName(), "Robot", // required interface
                 device->GetName(), "Main"); // provided interface
manager->CreateAll();
manager->StartAll();
```
At runtime, the `Run` method from `mtsROSBridge` will be called periodically.  At each call, the bridge will:
* read the data using the *C++/cisst* data type used by the device (e.g. `std::string` in example above)
* convert the data from *cisst* to *ROS* format using the include/cisst_ros_bridge/mtsROSBridge.hoverloaded global function `mtsCISSTToROS`.  The function `mtsCISSTToROS` also converts the timestamps (if any) and checks if the *cisst* data is valid (if the *cisst* data type has a `Valid` flag)
* if the data can be converted to *ROS* (e.g. `std_msgs::String`) and is valid, it is published on the ROS topic defined in `AddPublisherFromCommandRead` (in this example, "/my_device/status")

The most common conversions are implemented in the `cisst_ros_bridge` package.  They are declared in the file ` cisst_ros_bridge/include/cisst_ros_bridge/mtsCISSTToROS.h`.  If you need to add your own conversions (i.e. overload the global function `mtsCISSTTOROS`), make sure you include (as in `#include`) your overloaded definitions before including the `mtsROSBridge.h` header file.  Otherwise the compiler will ignore your overloaded definitions.

You can also add a ROS subscriber to send commands to the device.  In the following example, the bridge will add a subscriber for the topic "/my_device/shutdown".  There is no template parameter since the command "Shutdown" is a void command and doesn't use any parameter.
```c++
bridge->AddSubscriberToCommandVoid
    ("Robot", "Shutdown", "/my_device/shutdown");
```

If you need to send a payload to the device, you can use the method `AddSubscriberToCommandWrite`.  For example, if your robot has a command called "Connect" that expects a `std::string` to encode an IP address, you can add a ROS subscriber that will use ROS messages of type `std_msgs::String`.  Internally the ROS bridge will convert the *ROS* payload to a *cisst* payload using the overload global function `mtsROSToCISST` (similar to `mtsCISSTToROS` mentioned above).
```c++
bridge->AddSubscriberToCommandWrite<std::string, std_msgs::String>
    ("Robot", "Connect", "/my_device/connect");
```

The class `mtsROSBridge` (see header file `include/cisst_ros_bridge/mtsROSBridge.h`) also has methods to bridge:
* *cisst* events to ROS topics
* *cisst* commands to ROS services
* *cisst* commands and events to ROS tf2
* populate *cisst* provided interfaces as well as required interfaces

**Note re. timing:**  the class `mtsROSBridge` is derived from the *cisstMultiTask* class `mtsTaskPeriodic`.  When constructing an instance of the ROS bridge (or class derived from `mtsROSBridge`), you have to provide the periodicity used to call the internal `Run` method.  This period will determine how often the publishers from read commands will publish but **also** when the events and commands will be de-queued.  So if you set your bridge's period to 1 second, the bridge will only process the ROS subscribers every second.  This means that your message sent from ROS might be waiting for a full second before being send to your device.  To avoid this, you can create multiple ROS bridges.  For most applications, you will likely need:
* one bridge for all data published at the same constant periodicity (`AddPublisherFromCommandRead`)
* one bridge with a very short period (for example 0.1 millisecond) to handle all the publishers from *cisst* events and subscribers
* one bridge for ROS tf2 (if used)

## cisst ROS CRTK bridge

# Examples

The cisst-ros package is used by the following components to create ROS interfaces:
* dVRK: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki (da Vinci Research Kit)
* sawNDITracker: https://github.com/jhu-saw/sawNDITracker (NDI serial port based optical and magnetic trackers)
* sawForceDimensionSDK: https://github.com/jhu-saw/sawForceDimensionSDK (Force Dimension haptic devices and Novint Falcon)
* ... and many more cisst/SAW components found in https://github.com/jhu-saw

