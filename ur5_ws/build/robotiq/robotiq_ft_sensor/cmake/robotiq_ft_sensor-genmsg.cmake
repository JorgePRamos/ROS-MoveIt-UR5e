# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robotiq_ft_sensor: 1 messages, 1 services")

set(MSG_I_FLAGS "-Irobotiq_ft_sensor:/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotiq_ft_sensor_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" NAME_WE)
add_custom_target(_robotiq_ft_sensor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotiq_ft_sensor" "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" ""
)

get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" NAME_WE)
add_custom_target(_robotiq_ft_sensor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotiq_ft_sensor" "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robotiq_ft_sensor
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Services
_generate_srv_cpp(robotiq_ft_sensor
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Module File
_generate_module_cpp(robotiq_ft_sensor
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_ft_sensor
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotiq_ft_sensor_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotiq_ft_sensor_generate_messages robotiq_ft_sensor_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_cpp _robotiq_ft_sensor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_cpp _robotiq_ft_sensor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_ft_sensor_gencpp)
add_dependencies(robotiq_ft_sensor_gencpp robotiq_ft_sensor_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_ft_sensor_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robotiq_ft_sensor
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Services
_generate_srv_eus(robotiq_ft_sensor
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Module File
_generate_module_eus(robotiq_ft_sensor
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_ft_sensor
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robotiq_ft_sensor_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robotiq_ft_sensor_generate_messages robotiq_ft_sensor_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_eus _robotiq_ft_sensor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_eus _robotiq_ft_sensor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_ft_sensor_geneus)
add_dependencies(robotiq_ft_sensor_geneus robotiq_ft_sensor_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_ft_sensor_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robotiq_ft_sensor
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Services
_generate_srv_lisp(robotiq_ft_sensor
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Module File
_generate_module_lisp(robotiq_ft_sensor
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_ft_sensor
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robotiq_ft_sensor_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robotiq_ft_sensor_generate_messages robotiq_ft_sensor_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_lisp _robotiq_ft_sensor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_lisp _robotiq_ft_sensor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_ft_sensor_genlisp)
add_dependencies(robotiq_ft_sensor_genlisp robotiq_ft_sensor_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_ft_sensor_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robotiq_ft_sensor
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Services
_generate_srv_nodejs(robotiq_ft_sensor
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Module File
_generate_module_nodejs(robotiq_ft_sensor
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_ft_sensor
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robotiq_ft_sensor_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robotiq_ft_sensor_generate_messages robotiq_ft_sensor_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_nodejs _robotiq_ft_sensor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_nodejs _robotiq_ft_sensor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_ft_sensor_gennodejs)
add_dependencies(robotiq_ft_sensor_gennodejs robotiq_ft_sensor_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_ft_sensor_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robotiq_ft_sensor
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Services
_generate_srv_py(robotiq_ft_sensor
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Module File
_generate_module_py(robotiq_ft_sensor
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotiq_ft_sensor_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotiq_ft_sensor_generate_messages robotiq_ft_sensor_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_py _robotiq_ft_sensor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_py _robotiq_ft_sensor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_ft_sensor_genpy)
add_dependencies(robotiq_ft_sensor_genpy robotiq_ft_sensor_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_ft_sensor_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_ft_sensor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_ft_sensor
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robotiq_ft_sensor_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_ft_sensor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_ft_sensor
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robotiq_ft_sensor_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_ft_sensor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_ft_sensor
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robotiq_ft_sensor_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_ft_sensor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_ft_sensor
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robotiq_ft_sensor_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robotiq_ft_sensor_generate_messages_py std_msgs_generate_messages_py)
endif()
