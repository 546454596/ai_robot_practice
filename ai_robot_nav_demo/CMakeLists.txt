cmake_minimum_required(VERSION 3.0.2)
project(ai_robot_nav_demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED 
COMPONENTS
  amcl
  map_server
  move_base
  tf
  tf2
  tf2_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  actionlib
  actionlib_msgs
  rostime
  sensor_msgs
  message_filters
  cv_bridge
  image_transport
  compressed_image_transport
  compressed_depth_image_transport
  geometry_msgs
)

catkin_package()

set(BOOST_INCLUDEDIR "/usr/include/boost")
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")
find_package(Boost 1.65.1 REQUIRED COMPONENTS chrono regex thread filesystem log system)

if (Boost_FOUND)
	message(STATUS ">>>> ${Boost_ROOT}")
	message(STATUS ">>>> ${Boost_INCLUDE_DIRS}")
	message(STATUS ">>>> ${Boost_LIBRARIES}")
endif()


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/nav_demo_node.cpp)
add_executable(mapbasetf src/mapbasetf.cpp)
target_link_libraries(mapbasetf ${catkin_LIBRARIES})

add_executable(Odom_test src/Odom_test.cpp)
if(nav_msgs_EXPORTED_TARGETS)
  add_dependencies(Odom_test ${nav_msgs_EXPORTED_TARGETS})
endif()
target_link_libraries(Odom_test ${catkin_LIBRARIES})
install(TARGETS Odom_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(move_object src/move_object.cpp)
target_link_libraries(move_object
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#add_executable(initialpose src/initialpose)
#target_link_libraries(initialpose ${catkin_LIBRARIES})
