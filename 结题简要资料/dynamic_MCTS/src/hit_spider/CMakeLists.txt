cmake_minimum_required(VERSION 3.0.2)
project(hit_spider)
SET(CMAKE_BUILD_TYPE Debug)


add_definitions("-Wall")  #启用g++检测代码警告

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs

  tf2
  tf2_ros
  tf2_geometry_msgs

  message_generation

  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
)

## System dependencies are found with CMake's conventions
#Eigen3路径
find_package(Eigen3 REQUIRED)
if (EIGEN3_INCLUDE_DIR)
	message(STATUS "Found Eigen3")
	include_directories("${EIGEN3_INCLUDE_DIR}")	#包含Eigen库头文件
endif()

#GLPK路径
set(GLPK_INCLUDE_DIR "/usr/local/include")
set(GLPK_LIBRARY_DIR "/usr/local/lib")
include_directories("${GLPK_INCLUDE_DIR}")	#包含GLPK头文件
link_directories("${GLPK_LIBRARY_DIR}")			#GLPK的链接库

#cdd库
set(CDD_INCLUDE_DIR "/usr/include/cdd")
set(CDD_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu/libcdd.so")
include_directories("${CDD_INCLUDE_DIR}")	  #包含cdd的头文件
link_directories("${CDD_LIBRARY_DIR}")			#cdd库的链接库

add_message_files(FILES hexapod_RPY.msg hexapod_Base_Pose.msg FeetPosition.msg hexapod_State.msg)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES hit_spider
 CATKIN_DEPENDS message_runtime
 DEPENDS EIGEN3
)


include_directories(include ${catkin_INCLUDE_DIRS})

##---规划库的构建---##
set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}")	#工程的include文件夹
set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")		#工程的src文件夹
set(lib_Bisic bisic_library)
set(lib_MCTS dynamic_MCTS_library)
set(lib_RST library_robot_state_transition)
#头文件
set(lib_Bisic_headers
		${INCLUDE_DIR}/util.hh
		${INCLUDE_DIR}/hexapod.hh
		${INCLUDE_DIR}/static_information.hh
)
set(lib_RST_headers
    ${INCLUDE_DIR}/robot_state_transition/solve_LP_GLPK.hh
    ${INCLUDE_DIR}/robot_state_transition/Bretl.hh
    ${INCLUDE_DIR}/robot_state_transition/my_cdd.hh
    ${INCLUDE_DIR}/robot_state_transition/kinematics_constrain.hh
    ${INCLUDE_DIR}/robot_state_transition/dynamic_constrain.hh
)
set(lib_MCTS_headers
    ${INCLUDE_DIR}/MCTS/planning.hh
    ${INCLUDE_DIR}/MCTS/MTree.hh
)
#源文件
set(lib_Bisic_sources
    ${lib_Bisic_headers}
		${SRC_DIR}/util.cpp
		${SRC_DIR}/hexapod.cpp
    ${SRC_DIR}/static_information.cpp
)
set(lib_RST_sources
    ${lib_RST_headers}
    ${SRC_DIR}/robot_state_transition/solve_LP_GLPK.cpp
    ${SRC_DIR}/robot_state_transition/Bretl.cpp
    ${SRC_DIR}/robot_state_transition/my_cdd.cpp
    ${SRC_DIR}/robot_state_transition/kinematics_constrain.cpp
    ${SRC_DIR}/robot_state_transition/dynamic_constrain.cpp
)
set(lib_MCTS_sources
    ${lib_MCTS_headers}
    ${SRC_DIR}/MCTS/planning.cpp
    ${SRC_DIR}/MCTS/MTree.cpp
)

add_library(${lib_Bisic} SHARED ${lib_Bisic_sources})		#构建 基本链接库  整机逆运动学、地图环境落足点
add_dependencies(${lib_Bisic} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${lib_Bisic} ${catkin_LIBRARIES})	#链接外部库

add_library(${lib_RST} SHARED ${lib_RST_sources})		#构建  机器人状态转移  动态链接库
add_dependencies(${lib_RST} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${lib_RST} ${lib_Bisic} libglpk.so ${CDD_LIBRARY_DIR} ${catkin_LIBRARIES})  #链接外部库

add_library(${lib_MCTS} SHARED ${lib_MCTS_sources})		#构建  蒙特卡洛搜索树算法  动态链接库
add_dependencies(${lib_MCTS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${lib_MCTS} ${lib_Bisic} ${lib_RST} ${catkin_LIBRARIES})  #链接外部库
##---规划库的构建---##

#------可执行程序------
add_executable(static_information_publisher src/static_information_publisher.cpp)
target_link_libraries(static_information_publisher ${lib_Bisic})

add_executable(main src/main.cpp)
target_link_libraries(main ${lib_MCTS})


add_compile_options(-fPIC)