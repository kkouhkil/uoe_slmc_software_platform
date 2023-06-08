cmake_minimum_required(VERSION 2.8.3)
project(tf_server)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  tf2
  tf2_ros
)

catkin_package(
  LIBRARIES tf_server
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf tf2 tf2_ros
  #DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odo_to_tf2_node src/odo_to_tf2_node.cpp)
target_link_libraries(odo_to_tf2_node
 ${catkin_LIBRARIES}
)

add_executable(odo_to_tf_node src/odo_to_tf_node.cpp)
target_link_libraries(odo_to_tf_node
 ${catkin_LIBRARIES}
)

install(TARGETS odo_to_tf2_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


