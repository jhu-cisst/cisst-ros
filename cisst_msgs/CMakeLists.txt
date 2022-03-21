cmake_minimum_required(VERSION 3.1)
project(cisst_msgs VERSION 2.0.0)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  )


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  vctDoubleVec.msg
  prmCartesianImpedanceGains.msg
  mtsIntervalStatistics.msg
  BoolStamped.msg
  StringStamped.msg
  )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  QueryForwardKinematics.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  sensor_msgs
  )


catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES cisst_ros_bridge
  CATKIN_DEPENDS message_runtime
  #  DEPENDS system_lib
)

###########
## Build ##
###########

# include_directories(include)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )
