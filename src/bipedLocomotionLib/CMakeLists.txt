cmake_minimum_required(VERSION 2.8.3)
project(biped_locomotion_lib)

# required packages
find_package(catkin REQUIRED COMPONENTS sl_config sl floating_base_tools)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)
sl_find_package_GLUT()
find_package(PNG REQUIRED)

# find optional packages
find_package(xenomai_ros QUIET)

catkin_package(
  INCLUDE_DIRS src/task/controllers src/task/utils src/task/tasks src/opengl
  CATKIN_DEPENDS sl floating_base_tools
)

set(BUILD_DOCUMENTATION OFF)
#set(CMAKE_BUILD_TYPE Debug)

if (Xenomai_FOUND)
  add_xenomai_flags()
endif()

include_directories(
  src/opengl
  src/task/controllers
  src/task/tasks
  src/task/utils
  ${Eigen_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PNG_INCLUDE_DIR}
  ${py_momentum_mpc_INCLUDE_DIRS}
  ${Xenomai_INCLUDE_DIR}
)

#
# build libraries for each robot
#
FOREACH(ROB ${floating_base_tools_ROBOTS})
  message(STATUS "building biped_locomotion libraries for ${ROB}")
  add_library(biped_locomotion_lib_${ROB}
    src/task/utils/BipedalTaskComposer.cpp
    src/task/utils/BipedalTaskComposer_precompiles.cpp
    src/task/utils/capture_point_helper.cpp
    src/task/utils/FootContactHandlerHermes.cpp
    src/task/utils/min_jerk_generator.cpp
    src/task/utils/quaternions.cpp
    src/task/utils/inverse_kinematics.cpp
    src/task/controllers/biped_controller.cpp
    src/task/controllers/balance_controller.cpp
    src/task/controllers/walking_controller.cpp
    src/task/tasks/zero_torque_task.cpp
    src/task/tasks/torque_sine_task.cpp
    #src/task/tasks/wholebody_task.cpp
    src/task/tasks/mass_estimation.cpp
    src/task/tasks/gravity_compensation.cpp
    src/task/tasks/fb_sine_task.cpp
    src/task/tasks/cog_force_utils.cpp
    src/task/tasks/balance_task.cpp
    src/task/tasks/walking_task.cpp
    src/task/tasks/joint_calibration_task.cpp
    src/task/tasks/test_inverse_kinematics.cpp
    src/task/tasks/initUserTasks_bipedLocomotionLib.c
    # example hinvdyn task
    #src/task/controllers/HInvDynExample.cpp
    #src/task/tasks/HInvDynExample_task.cpp
    # mock up a new task
    src/task/controllers/mytest.cpp
    src/task/tasks/wholebody_task.cpp
    src/task/tasks/mytest_task.cpp
    src/task/controllers/ArmReflex.cpp
  )
  target_link_libraries(biped_locomotion_lib_${ROB} 
    ${ROB}
    ${floating_base_tools_LIBRARIES}
    floating_base_tools_${ROB}
    ${Xenomai_LIBRARIES_NATIVE}
    ${Xenomai_LIBRARY_RTDM}
  )
  message(STATUS "floating_base_tools_DEFINITIONS: ${floating_base_tools_DEFINITIONS}")
  set_target_properties(biped_locomotion_lib_${ROB} PROPERTIES COMPILE_FLAGS "-I${${ROB}_SOURCE_PREFIX}/include" COMPILE_DEFINITIONS "${floating_base_tools_DEFINITIONS}")

  add_library(biped_locomotion_lib_openGL_${ROB}
    src/opengl/capture_opengl.cpp
    src/opengl/drawUtils.cpp
    src/opengl/display_kinematics.c
  )
  target_link_libraries(biped_locomotion_lib_openGL_${ROB} 
    SLopenGL
    ${PNG_LIBRARIES}
    ${ROB}
    ${Xenomai_LIBRARIES_NATIVE}
    ${Xenomai_LIBRARY_RTDM}
  )
  set_target_properties(biped_locomotion_lib_openGL_${ROB} PROPERTIES COMPILE_FLAGS "-I${${ROB}_SOURCE_PREFIX}/include") 
ENDFOREACH(ROB)

# add a target to generate API documentation with Doxygen
if(BUILD_DOCUMENTATION)
  message(STATUS "building doxygen documentation for ${PROJECT_NAME}")
  FIND_PACKAGE(Doxygen)
  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR
      "Doxygen is needed to build the documentation. Please install it correctly")
  endif()
configure_file(doc/Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)
add_custom_target (${PROJECT_NAME}_doc ALL
COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)

endif()
