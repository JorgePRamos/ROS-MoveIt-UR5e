# Install script for directory: /home/bicrobotics/UR5e/ur5_ws/src/robotiq/robotiq_3f_gripper_articulated_gazebo_plugins

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_3f_gripper_articulated_gazebo_plugins/catkin_generated/installspace/robotiq_3f_gripper_articulated_gazebo_plugins.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_3f_gripper_articulated_gazebo_plugins/cmake" TYPE FILE FILES
    "/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_3f_gripper_articulated_gazebo_plugins/catkin_generated/installspace/robotiq_3f_gripper_articulated_gazebo_pluginsConfig.cmake"
    "/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_3f_gripper_articulated_gazebo_plugins/catkin_generated/installspace/robotiq_3f_gripper_articulated_gazebo_pluginsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_3f_gripper_articulated_gazebo_plugins" TYPE FILE FILES "/home/bicrobotics/UR5e/ur5_ws/src/robotiq/robotiq_3f_gripper_articulated_gazebo_plugins/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_articulated_gazebo_plugins/plugins/libRobotiqHandPlugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_articulated_gazebo_plugins/plugins/libRobotiqHandPlugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_articulated_gazebo_plugins/plugins/libRobotiqHandPlugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_articulated_gazebo_plugins/plugins" TYPE SHARED_LIBRARY FILES "/home/bicrobotics/UR5e/ur5_ws/devel/lib/libRobotiqHandPlugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_articulated_gazebo_plugins/plugins/libRobotiqHandPlugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_articulated_gazebo_plugins/plugins/libRobotiqHandPlugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_articulated_gazebo_plugins/plugins/libRobotiqHandPlugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robotiq_3f_gripper_articulated_gazebo_plugins/plugins/libRobotiqHandPlugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

