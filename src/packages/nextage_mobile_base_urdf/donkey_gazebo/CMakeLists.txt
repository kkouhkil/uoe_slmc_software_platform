cmake_minimum_required(VERSION 2.8.3)
project(donkey_gazebo)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package()

file(GLOB xacro_files urdf/*.urdf.xacro)
xacro_add_files(${xacro_files} TARGET urdf INSTALL)

if(CATKIN_ENABLE_TESTING)
endif(CATKIN_ENABLE_TESTING)

install(DIRECTORY config urdf launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)