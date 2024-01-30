cmake_minimum_required(VERSION 3.10)
project(cisst_msgs VERSION 2.2.0)

# first test for ROS1
find_package (catkin QUIET
  COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  )

# ROS 1
if (catkin_FOUND)

  ## Generate messages in the 'msg' folder
  add_message_files (FILES
    DoubleVec.msg
    IntervalStatistics.msg
    BoolStamped.msg
    )

  ## Generate services in the 'srv' folder
  add_service_files (FILES
    ConvertFloat64Array.srv
    )

  ## Generate added messages and services with any dependencies listed here
  generate_messages (DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    geometry_msgs
    sensor_msgs
    )

  catkin_package (
    CATKIN_DEPENDS message_runtime
    )


else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    add_definitions (-DROS2)
    set (ROS_VERSION 2)
    find_package (ament_cmake REQUIRED)
    find_package (rclcpp REQUIRED)
    find_package (std_msgs REQUIRED)
    find_package (geometry_msgs REQUIRED)
    find_package (builtin_interfaces REQUIRED)
    find_package (rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces (cisst_msgs
      msg/DoubleVec.msg
      msg/IntervalStatistics.msg
      msg/BoolStamped.msg
      srv/ConvertFloat64Array.srv
      DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
      )

    ament_package ()

  endif (ament_cmake_FOUND)

endif (catkin_FOUND)
