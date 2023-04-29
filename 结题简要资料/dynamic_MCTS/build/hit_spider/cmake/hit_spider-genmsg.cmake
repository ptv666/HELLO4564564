# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hit_spider: 4 messages, 0 services")

set(MSG_I_FLAGS "-Ihit_spider:/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hit_spider_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg" NAME_WE)
add_custom_target(_hit_spider_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hit_spider" "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg" ""
)

get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg" NAME_WE)
add_custom_target(_hit_spider_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hit_spider" "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg" "hit_spider/hexapod_RPY:geometry_msgs/Point"
)

get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg" NAME_WE)
add_custom_target(_hit_spider_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hit_spider" "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg" "hit_spider/FeetPosition:hit_spider/hexapod_Base_Pose:std_msgs/String:hit_spider/hexapod_RPY:geometry_msgs/Point"
)

get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg" NAME_WE)
add_custom_target(_hit_spider_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hit_spider" "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hit_spider
)
_generate_msg_cpp(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg"
  "${MSG_I_FLAGS}"
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hit_spider
)
_generate_msg_cpp(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg"
  "${MSG_I_FLAGS}"
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg;/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hit_spider
)
_generate_msg_cpp(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hit_spider
)

### Generating Services

### Generating Module File
_generate_module_cpp(hit_spider
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hit_spider
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hit_spider_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hit_spider_generate_messages hit_spider_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_cpp _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_cpp _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_cpp _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_cpp _hit_spider_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hit_spider_gencpp)
add_dependencies(hit_spider_gencpp hit_spider_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hit_spider_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hit_spider
)
_generate_msg_eus(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg"
  "${MSG_I_FLAGS}"
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hit_spider
)
_generate_msg_eus(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg"
  "${MSG_I_FLAGS}"
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg;/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hit_spider
)
_generate_msg_eus(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hit_spider
)

### Generating Services

### Generating Module File
_generate_module_eus(hit_spider
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hit_spider
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hit_spider_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hit_spider_generate_messages hit_spider_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_eus _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_eus _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_eus _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_eus _hit_spider_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hit_spider_geneus)
add_dependencies(hit_spider_geneus hit_spider_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hit_spider_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hit_spider
)
_generate_msg_lisp(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg"
  "${MSG_I_FLAGS}"
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hit_spider
)
_generate_msg_lisp(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg"
  "${MSG_I_FLAGS}"
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg;/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hit_spider
)
_generate_msg_lisp(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hit_spider
)

### Generating Services

### Generating Module File
_generate_module_lisp(hit_spider
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hit_spider
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hit_spider_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hit_spider_generate_messages hit_spider_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_lisp _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_lisp _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_lisp _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_lisp _hit_spider_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hit_spider_genlisp)
add_dependencies(hit_spider_genlisp hit_spider_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hit_spider_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hit_spider
)
_generate_msg_nodejs(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg"
  "${MSG_I_FLAGS}"
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hit_spider
)
_generate_msg_nodejs(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg"
  "${MSG_I_FLAGS}"
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg;/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hit_spider
)
_generate_msg_nodejs(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hit_spider
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hit_spider
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hit_spider
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hit_spider_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hit_spider_generate_messages hit_spider_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_nodejs _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_nodejs _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_nodejs _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_nodejs _hit_spider_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hit_spider_gennodejs)
add_dependencies(hit_spider_gennodejs hit_spider_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hit_spider_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hit_spider
)
_generate_msg_py(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg"
  "${MSG_I_FLAGS}"
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hit_spider
)
_generate_msg_py(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg"
  "${MSG_I_FLAGS}"
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg;/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hit_spider
)
_generate_msg_py(hit_spider
  "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hit_spider
)

### Generating Services

### Generating Module File
_generate_module_py(hit_spider
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hit_spider
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hit_spider_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hit_spider_generate_messages hit_spider_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_RPY.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_py _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_Base_Pose.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_py _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/hexapod_State.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_py _hit_spider_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ptw/__BiYeSheJi/dynamic_MCTS/src/hit_spider/msg/FeetPosition.msg" NAME_WE)
add_dependencies(hit_spider_generate_messages_py _hit_spider_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hit_spider_genpy)
add_dependencies(hit_spider_genpy hit_spider_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hit_spider_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hit_spider)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hit_spider
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(hit_spider_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hit_spider_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hit_spider)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hit_spider
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(hit_spider_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hit_spider_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hit_spider)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hit_spider
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(hit_spider_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hit_spider_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hit_spider)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hit_spider
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(hit_spider_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hit_spider_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hit_spider)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hit_spider\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hit_spider
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(hit_spider_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hit_spider_generate_messages_py std_msgs_generate_messages_py)
endif()
