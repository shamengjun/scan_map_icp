cmake_minimum_required(VERSION 2.8.3)
project(scan_map_icp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  tf
  cmake_modules
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scan_map_icp
#  CATKIN_DEPENDS laser_geometry nav_msgs pcl_conversions pcl_ros roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 add_library(${PROJECT_NAME}
   src/scan_map_icp.cpp
 )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(${PROJECT_NAME}_node src/scan_map_icp_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

