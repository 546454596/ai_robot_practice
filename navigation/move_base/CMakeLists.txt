cmake_minimum_required(VERSION 3.1)
project(move_base)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        clear_costmap_recovery
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_generation
        nav_core
        navfn

        pluginlib
        roscpp
        rospy
        rotate_recovery
        std_srvs
        tf2_geometry_msgs
        tf2_ros

        cv_bridge
        roscpp
        rospy
        std_srvs
        std_msgs
        pcl_conversions
        pcl_ros
        
        ai_robot_interface
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveBase.cfg
)

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# move_base
add_library(move_base
  src/move_base.cpp
)
target_link_libraries(move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_base_node
  src/move_base_node.cpp
)
add_dependencies(move_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_node move_base)
set_target_properties(move_base_node PROPERTIES OUTPUT_NAME move_base)

add_executable(obs_avoid src/obsavoid/obs_avoid.cpp)
add_dependencies(obs_avoid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obs_avoid
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )

#add_executable(ai_robot_ranger src/ai_robot_ranger.cpp)
#add_executable(multi_point_inspection src/multi_point_inspection/multi_point_inspection.cpp)
add_executable(ai_robot_modify
  src/main.cpp
  src/AIBrain.cpp
  src/assistMath.cpp
  src/posControl.cpp
  src/Pioneer3AT.cpp
  src/pointtopoint/PointToPoint.cpp
  src/Pioneer3AT.cpp
  src/postrajectory.cpp
  src/PIDController.cpp
  src/pid_fuzzy.cpp
  src/ADRC.cpp
  src/ARDrone.cpp
  src/drone_object_ros.cpp
  src/move_base.cpp
  src/ESO.cpp
)
target_link_libraries(ai_robot_modify
        move_base
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        )

install(
    TARGETS
        move_base_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        move_base
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation 安装目录的时候默认会安装所有的文件，如果使用FILES_MATCHING关键字（在第一个PATTERN或者REGEX之前），则表示必须要满足对应的模式或者正则的文件才能被安装。
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
