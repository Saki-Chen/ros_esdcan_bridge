cmake_minimum_required(VERSION 3.0.2)
project(ros_esdcan_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES can_io
 DEPENDS system_lib -lntcan
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(can_io src/can_io.cpp)
target_link_libraries(can_io -lntcan -ljsoncpp)

add_executable(ros_esdcan_bridge src/ros_esdcan_bridge.cpp)
target_link_libraries(ros_esdcan_bridge can_io ${catkin_LIBRARIES})

install(TARGETS ros_esdcan_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS can_io
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
