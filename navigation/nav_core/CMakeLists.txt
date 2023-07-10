cmake_minimum_required(VERSION 3.1)
project(nav_core)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
        COMPONENTS
            std_msgs
            geometry_msgs
            tf2_ros
            costmap_2d
        )

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
            std_msgs
            geometry_msgs
            tf2_ros
            costmap_2d
)


## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
