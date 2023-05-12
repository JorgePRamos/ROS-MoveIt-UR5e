# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robotiq_2f_gripper_control: 2 messages, 0 services")

set(MSG_I_FLAGS "-Irobotiq_2f_gripper_control:/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotiq_2f_gripper_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg" NAME_WE)
add_custom_target(_robotiq_2f_gripper_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotiq_2f_gripper_control" "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg" ""
)

get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg" NAME_WE)
add_custom_target(_robotiq_2f_gripper_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotiq_2f_gripper_control" "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robotiq_2f_gripper_control
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_2f_gripper_control
)
_generate_msg_cpp(robotiq_2f_gripper_control
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_2f_gripper_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(robotiq_2f_gripper_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_2f_gripper_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotiq_2f_gripper_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotiq_2f_gripper_control_generate_messages robotiq_2f_gripper_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg" NAME_WE)
add_dependencies(robotiq_2f_gripper_control_generate_messages_cpp _robotiq_2f_gripper_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg" NAME_WE)
add_dependencies(robotiq_2f_gripper_control_generate_messages_cpp _robotiq_2f_gripper_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_2f_gripper_control_gencpp)
add_dependencies(robotiq_2f_gripper_control_gencpp robotiq_2f_gripper_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_2f_gripper_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robotiq_2f_gripper_control
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_2f_gripper_control
)
_generate_msg_eus(robotiq_2f_gripper_control
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_2f_gripper_control
)

### Generating Services

### Generating Module File
_generate_module_eus(robotiq_2f_gripper_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_2f_gripper_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robotiq_2f_gripper_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robotiq_2f_gripper_control_generate_messages robotiq_2f_gripper_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg" NAME_WE)
add_dependencies(robotiq_2f_gripper_control_generate_messages_eus _robotiq_2f_gripper_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg" NAME_WE)
add_dependencies(robotiq_2f_gripper_control_generate_messages_eus _robotiq_2f_gripper_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_2f_gripper_control_geneus)
add_dependencies(robotiq_2f_gripper_control_geneus robotiq_2f_gripper_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_2f_gripper_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robotiq_2f_gripper_control
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_2f_gripper_control
)
_generate_msg_lisp(robotiq_2f_gripper_control
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_2f_gripper_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(robotiq_2f_gripper_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_2f_gripper_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robotiq_2f_gripper_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robotiq_2f_gripper_control_generate_messages robotiq_2f_gripper_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg" NAME_WE)
add_dependencies(robotiq_2f_gripper_control_generate_messages_lisp _robotiq_2f_gripper_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg" NAME_WE)
add_dependencies(robotiq_2f_gripper_control_generate_messages_lisp _robotiq_2f_gripper_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_2f_gripper_control_genlisp)
add_dependencies(robotiq_2f_gripper_control_genlisp robotiq_2f_gripper_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_2f_gripper_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robotiq_2f_gripper_control
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_2f_gripper_control
)
_generate_msg_nodejs(robotiq_2f_gripper_control
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_2f_gripper_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(robotiq_2f_gripper_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_2f_gripper_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robotiq_2f_gripper_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robotiq_2f_gripper_control_generate_messages robotiq_2f_gripper_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg" NAME_WE)
add_dependencies(robotiq_2f_gripper_control_generate_messages_nodejs _robotiq_2f_gripper_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg" NAME_WE)
add_dependencies(robotiq_2f_gripper_control_generate_messages_nodejs _robotiq_2f_gripper_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_2f_gripper_control_gennodejs)
add_dependencies(robotiq_2f_gripper_control_gennodejs robotiq_2f_gripper_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_2f_gripper_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robotiq_2f_gripper_control
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_2f_gripper_control
)
_generate_msg_py(robotiq_2f_gripper_control
  "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_2f_gripper_control
)

### Generating Services

### Generating Module File
_generate_module_py(robotiq_2f_gripper_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_2f_gripper_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotiq_2f_gripper_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotiq_2f_gripper_control_generate_messages robotiq_2f_gripper_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg" NAME_WE)
add_dependencies(robotiq_2f_gripper_control_generate_messages_py _robotiq_2f_gripper_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bicrobotics/ROS-MoveIt-UR5e/ur5_ws/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg" NAME_WE)
add_dependencies(robotiq_2f_gripper_control_generate_messages_py _robotiq_2f_gripper_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_2f_gripper_control_genpy)
add_dependencies(robotiq_2f_gripper_control_genpy robotiq_2f_gripper_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_2f_gripper_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_2f_gripper_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_2f_gripper_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_2f_gripper_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotiq_2f_gripper_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_2f_gripper_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotiq_2f_gripper_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_2f_gripper_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotiq_2f_gripper_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_2f_gripper_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_2f_gripper_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_2f_gripper_control
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_2f_gripper_control")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_2f_gripper_control
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
