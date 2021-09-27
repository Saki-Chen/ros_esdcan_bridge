cmake_minimum_required(VERSION 3.0.2)
project(ros_esdcan_bridge)

option(BUILD_CAN_IO "build libcan_io with ntcan support" ON)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
)

find_package(jsoncpp REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(${BUILD_CAN_IO})
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES can_io can_coder
    CATKIN_DEPENDS roscpp can_msgs
    DEPENDS -ljsoncpp -lntcan
  )

  add_library(can_io src/can_io.cpp)
  target_link_libraries(can_io -lntcan)

  add_executable(ros_esdcan_bridge nodes/ros_esdcan_bridge.cpp)
  target_link_libraries(ros_esdcan_bridge can_io ${catkin_LIBRARIES})
  add_dependencies(ros_esdcan_bridge ${catkin_EXPORTED_TARGETS})

  add_executable(powerUp others/powerUp.cpp)
  target_link_libraries(powerUp can_io)

  install(TARGETS ros_esdcan_bridge powerUp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(TARGETS can_io
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

else()
  catkin_package(
  INCLUDE_DIRS include
  LIBRARIES can_coder
  CATKIN_DEPENDS roscpp can_msgs
  DEPENDS -ljsoncpp
  )
endif()

add_library(can_coder src/can_coder.cpp)
target_link_libraries(can_coder -ljsoncpp)

add_executable(kvaser_powerUp others/kvaser_powerUp.cpp)
target_link_libraries(kvaser_powerUp  ${catkin_LIBRARIES})
add_dependencies(kvaser_powerUp ${catkin_EXPORTED_TARGETS})

install(TARGETS kvaser_powerUp
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS can_coder
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY 
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY 
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)