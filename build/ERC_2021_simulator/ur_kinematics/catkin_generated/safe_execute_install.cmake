execute_process(COMMAND "/home/diego/ERC_2021/build/ERC_2021_simulator/ur_kinematics/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/diego/ERC_2021/build/ERC_2021_simulator/ur_kinematics/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
