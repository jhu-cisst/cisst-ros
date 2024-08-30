cmake_minimum_required (VERSION 3.10)
project(cisst_ros_crtk VERSION 3.0.0)

# first test for ROS1
set (ROS1_depends
  roscpp
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  cisst_msgs
  cisst_ros_bridge
  crtk_msgs
  )

find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  include_directories (${catkin_INCLUDE_DIRS})
  catkin_package (
    INCLUDE_DIRS include
    LIBRARIES cisst_ros_crtk
    CATKIN_DEPENDS ${ROS1_depends}
    )

else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    set (ROS2_depends
      rclcpp
      std_msgs
      std_srvs
      geometry_msgs
      sensor_msgs
      diagnostic_msgs
      tf2_ros
      tf2_msgs
      cisst_msgs
      cisst_ros_bridge
      crtk_msgs)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()
  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

set(REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  )

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.3.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})
  include_directories (BEFORE include)

  # header
  set(HEADER_FILES
    include/cisst_ros_crtk/cisst_ros_crtk.h
    include/cisst_ros_crtk/mts_ros_crtk_bridge.h # for backward compatibility
    include/cisst_ros_crtk/mts_ros_crtk_bridge_provided.h
    include/cisst_ros_crtk/mts_ros_crtk_bridge_required.h
    include/cisst_ros_crtk/mtsCISSTToROS.h
    include/cisst_ros_crtk/mtsROSToCISST.h)

  # source
  set(SOURCE_FILES
    src/cisst_ros_crtk.cpp
    src/mts_ros_crtk_bridge_provided.cpp
    src/mts_ros_crtk_bridge_required.cpp
    src/mtsCISSTToROS.cpp
    src/mtsROSToCISST.cpp)

  add_library(cisst_ros_crtk ${HEADER_FILES} ${SOURCE_FILES})
  cisst_target_link_libraries (cisst_ros_crtk ${REQUIRED_CISST_LIBRARIES})

  if (ROS_VERSION EQUAL 1)
    target_link_libraries (cisst_ros_crtk ${catkin_LIBRARIES})

    install (TARGETS cisst_ros_crtk
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      )

    install (DIRECTORY include/${PROJECT_NAME}
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      )
  endif (ROS_VERSION EQUAL 1)

  if (ROS_VERSION EQUAL 2)
    ament_target_dependencies (cisst_ros_crtk ${ROS2_depends})

    install (TARGETS cisst_ros_crtk
      DESTINATION lib/${PROJECT_NAME}
      )

    install (DIRECTORY include/${PROJECT_NAME}
      DESTINATION include
      )

    install (
      TARGETS cisst_ros_crtk
      EXPORT cisst_ros_crtk
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include
      )

    ament_export_targets (cisst_ros_crtk HAS_LIBRARY_TARGET)

    ament_package ()

  endif (ROS_VERSION EQUAL 2)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
