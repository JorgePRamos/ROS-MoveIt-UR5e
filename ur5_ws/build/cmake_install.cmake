# Install script for directory: /home/bicrobotics/UR5e/ur5_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/bicrobotics/UR5e/ur5_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bicrobotics/UR5e/ur5_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bicrobotics/UR5e/ur5_ws/install" TYPE PROGRAM FILES "/home/bicrobotics/UR5e/ur5_ws/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bicrobotics/UR5e/ur5_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bicrobotics/UR5e/ur5_ws/install" TYPE PROGRAM FILES "/home/bicrobotics/UR5e/ur5_ws/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bicrobotics/UR5e/ur5_ws/install/setup.bash;/home/bicrobotics/UR5e/ur5_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bicrobotics/UR5e/ur5_ws/install" TYPE FILE FILES
    "/home/bicrobotics/UR5e/ur5_ws/build/catkin_generated/installspace/setup.bash"
    "/home/bicrobotics/UR5e/ur5_ws/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bicrobotics/UR5e/ur5_ws/install/setup.sh;/home/bicrobotics/UR5e/ur5_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bicrobotics/UR5e/ur5_ws/install" TYPE FILE FILES
    "/home/bicrobotics/UR5e/ur5_ws/build/catkin_generated/installspace/setup.sh"
    "/home/bicrobotics/UR5e/ur5_ws/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bicrobotics/UR5e/ur5_ws/install/setup.zsh;/home/bicrobotics/UR5e/ur5_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bicrobotics/UR5e/ur5_ws/install" TYPE FILE FILES
    "/home/bicrobotics/UR5e/ur5_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/bicrobotics/UR5e/ur5_ws/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/bicrobotics/UR5e/ur5_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/bicrobotics/UR5e/ur5_ws/install" TYPE FILE FILES "/home/bicrobotics/UR5e/ur5_ws/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/bicrobotics/UR5e/ur5_ws/build/gtest/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_2f_140_gripper_visualization/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_2f_85_gripper_visualization/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_2f_c2_gripper_visualization/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_3f_gripper_articulated_gazebo/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_3f_gripper_articulated_msgs/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/UR5_moveit_config/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/universal_robots/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/ur5e2f/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_ethercat/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_2f_gripper_control/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_ft_sensor/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_modbus_rtu/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_modbus_tcp/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_2f_gripper_action_server/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_3f_gripper_control/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_3f_gripper_joint_state_publisher/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/ur10_moveit_config/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/ur10e_moveit_config/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/ur16e_moveit_config/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/ur3_moveit_config/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/ur3e_moveit_config/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/ur5_moveit_config/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/ur5e_moveit_config/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/ur_description/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/ur_gazebo/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_3f_gripper_articulated_gazebo_plugins/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_3f_gripper_visualization/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_3f_rviz/cmake_install.cmake")
  include("/home/bicrobotics/UR5e/ur5_ws/build/universal_robot/ur_kinematics/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/bicrobotics/UR5e/ur5_ws/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
