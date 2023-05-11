execute_process(COMMAND "/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_2f_gripper_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bicrobotics/UR5e/ur5_ws/build/robotiq/robotiq_2f_gripper_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
