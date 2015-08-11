cmake_minimum_required(VERSION 2.8.3)
project(katana_pick_place_plugin)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_move_group
  dynamic_reconfigure
  roscpp
  rosconsole
  tf
  pluginlib
  actionlib
  cmake_modules
)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system filesystem date_time program_options)

generate_dynamic_reconfigure_options("katana_pick_place/cfg/KatanaPickPlaceDynamicReconfigure.cfg")

catkin_package(
  INCLUDE_DIRS
    katana_pick_place/include
    move_group_katana_pick_place_capability/include
  LIBRARIES
    moveit_katana_pick_place_planner
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    )

include_directories(katana_pick_place/include)
include_directories(move_group_katana_pick_place_capability/include)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(katana_pick_place)
add_subdirectory(move_group_katana_pick_place_capability)

install(FILES
  katana_pick_place_capability_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
