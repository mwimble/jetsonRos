cmake_minimum_required(VERSION 2.8.3)
project(kaimi_mid_camera)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake/)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  std_msgs
)

find_package(OpenCV REQUIRED core)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kaimi_mid_camera
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	cfg/cpp
  	${catkin_INCLUDE_DIRS}
  	${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

## Declare a C++ executable
add_executable(
	kaimi_mid_camera_node
	src/FindObject.cpp
	src/kaimi_mid_camera_node.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(
	kaimi_mid_camera_node
 	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(kaimi_mid_camera_node
  	${OpenCV_CV_LIBS}
	${catkin_LIBRARIES}
)

