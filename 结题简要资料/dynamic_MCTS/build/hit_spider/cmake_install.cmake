# Install script for directory: /home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ptw/__BiYeSheJi/dynamic_MCTS/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hit_spider/msg" TYPE FILE FILES
    "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg"
    "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg"
    "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg"
    "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hit_spider/cmake" TYPE FILE FILES "/home/ptw/__BiYeSheJi/dynamic_MCTS/build/hit_spider/catkin_generated/installspace/hit_spider-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ptw/__BiYeSheJi/dynamic_MCTS/devel/include/hit_spider")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ptw/__BiYeSheJi/dynamic_MCTS/devel/share/roseus/ros/hit_spider")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ptw/__BiYeSheJi/dynamic_MCTS/devel/share/common-lisp/ros/hit_spider")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ptw/__BiYeSheJi/dynamic_MCTS/devel/share/gennodejs/ros/hit_spider")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/ptw/__BiYeSheJi/dynamic_MCTS/devel/lib/python2.7/dist-packages/hit_spider")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/ptw/__BiYeSheJi/dynamic_MCTS/devel/lib/python2.7/dist-packages/hit_spider")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ptw/__BiYeSheJi/dynamic_MCTS/build/hit_spider/catkin_generated/installspace/hit_spider.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hit_spider/cmake" TYPE FILE FILES "/home/ptw/__BiYeSheJi/dynamic_MCTS/build/hit_spider/catkin_generated/installspace/hit_spider-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hit_spider/cmake" TYPE FILE FILES
    "/home/ptw/__BiYeSheJi/dynamic_MCTS/build/hit_spider/catkin_generated/installspace/hit_spiderConfig.cmake"
    "/home/ptw/__BiYeSheJi/dynamic_MCTS/build/hit_spider/catkin_generated/installspace/hit_spiderConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hit_spider" TYPE FILE FILES "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/package.xml")
endif()

