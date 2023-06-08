cmake_minimum_required(VERSION 3.0.2)
project(nextage_extensions)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf configs launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
catkin_install_python(PROGRAMS
    "scripts/force_sensor_interface"
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
