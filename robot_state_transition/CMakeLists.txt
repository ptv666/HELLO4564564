cmake_minimum_required(VERSION 3.0.2)
project(improve_CROC)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Debug")

# Eigen库
find_package(Eigen3 REQUIRED)
if (EIGEN3_INCLUDE_DIR)
	message(STATUS "Found Eigen3")
	include_directories("${EIGEN3_INCLUDE_DIR}")	#包含Eigen库头文件
endif()

# glpk的头文件路径
set(GLPK_INCLUDE_DIR "/usr/local/include")
set(GLPK_LIBRARY_DIR "/usr/local/lib")
include_directories("${GLPK_INCLUDE_DIR}")	#包含GLPK头文件
link_directories("${GLPK_LIBRARY_DIR}")			#GLPK的链接库

# 设置工程文件夹名称
set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")	#工程的include文件夹
set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")		#工程的src文件夹
set(TEST_DIR "${PROJECT_SOURCE_DIR}/test")


#########--------RST库的构建--------#########
set(RST_LIBRARY_NAME Robot_State_Transition)
# RST头文件
set(RST_HEADERS
		${INCLUDE_DIR}/util.hh
		${INCLUDE_DIR}/solve_LP_GLPK.hh
		${INCLUDE_DIR}/Bretl.hh
  )
# 工程的src文件夹
add_subdirectory(src)

# 工程的test文件夹
add_subdirectory(test)


# #设置为DEBUG模式
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb ")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_compile_options(-fPIC)