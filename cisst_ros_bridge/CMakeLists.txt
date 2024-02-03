cmake_minimum_required (VERSION 3.10)
project (cisst_ros_bridge VERSION 2.2.0)

# first test for ROS1
set (ROS1_depends
  roscpp
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  cisst_msgs
  )

find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS1 1)
  set (ROS2 0)
  set (ROS_VERSION 1)
  set (GENERATED_INCLUDE_DIR ${CATKIN_DEVEL_PREFIX}/include)
  file (MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})
  include_directories (${catkin_INCLUDE_DIRS})
  catkin_package (
    INCLUDE_DIRS include ${GENERATED_INCLUDE_DIR}
    LIBRARIES cisst_ros_bridge
    CATKIN_DEPENDS ${ROS1_depends}
    )

else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS1 0)
    set (ROS2 1)
    set (ROS_VERSION 2)
    set (GENERATED_INCLUDE_DIR ${cisst_ros_bridge_BINARY_DIR}/include)
    set (ROS2_depends
      rclcpp
      std_msgs
      std_srvs
      geometry_msgs
      sensor_msgs
      diagnostic_msgs
      tf2_ros
      tf2_msgs
      cisst_msgs)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()
  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  )

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})

  # configure header file
  configure_file (${cisst_ros_bridge_SOURCE_DIR}/src/cisst_ral_config.h.in
    ${GENERATED_INCLUDE_DIR}/cisst_ros_bridge/cisst_ral_config.h)

  include_directories (BEFORE include ${GENERATED_INCLUDE_DIR})

  # header
  set (HEADER_FILES
    include/cisst_ros_bridge/cisst_ral.h
    include/cisst_ros_bridge/mtsROSBridge.h
    include/cisst_ros_bridge/mtsCISSTToROS.h
    include/cisst_ros_bridge/mtsROSToCISST.h)

  # source
  set (SOURCE_FILES
    src/cisst_ral.cpp
    src/mtsROSBridge.cpp
    src/mtsCISSTToROS.cpp
    src/mtsROSToCISST.cpp)

  # library
  add_library (cisst_ros_bridge ${HEADER_FILES} ${SOURCE_FILES})
  cisst_target_link_libraries (cisst_ros_bridge ${REQUIRED_CISST_LIBRARIES})

  # example
  add_executable (example_bridge example/main.cpp)
  cisst_target_link_libraries (example_bridge ${REQUIRED_CISST_LIBRARIES})

  if (ROS_VERSION EQUAL 1)
    target_link_libraries (cisst_ros_bridge ${catkin_LIBRARIES})
    target_link_libraries (example_bridge cisst_ros_bridge ${catkin_LIBRARIES})

    install (TARGETS cisst_ros_bridge example_bridge
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      )

    install (DIRECTORY include/${PROJECT_NAME}
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      )

    install (DIRECTORY ${cisst_ros_bridge_BINARY_DIR}/include/${PROJECT_NAME}
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      )
  endif (ROS_VERSION EQUAL 1)

  if (ROS_VERSION EQUAL 2)
    ament_target_dependencies (cisst_ros_bridge
      ${ROS2_depends})

    ament_target_dependencies (example_bridge
      ${ROS2_depends})

    target_link_libraries (example_bridge cisst_ros_bridge)

    install (TARGETS cisst_ros_bridge example_bridge
      DESTINATION lib/${PROJECT_NAME}
      )

    install (DIRECTORY include/${PROJECT_NAME}
      DESTINATION include
      )

    install (DIRECTORY ${GENERATED_INCLUDE_DIR}/${PROJECT_NAME}
      DESTINATION include
      )

    install (
      TARGETS cisst_ros_bridge
      EXPORT cisst_ros_bridge
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include
      )

    ament_export_targets (cisst_ros_bridge HAS_LIBRARY_TARGET)

    ament_package ()

  endif (ROS_VERSION EQUAL 2)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
