cmake_minimum_required(VERSION 3.0.2)
project(ros_trees)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES ros_trees
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
  )

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# TODO probably do this...
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
