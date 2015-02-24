cmake_minimum_required(VERSION 2.8.3)
project(cisst_conversions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_conversions
  rospy
  tf
)

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  )
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf kdl_conversions
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  include/cisst_conversions/cisst_tf.h
  src/cisst_tf.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
cisst_target_link_libraries (${PROJECT_NAME} ${REQUIRED_CISST_LIBRARIES})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
