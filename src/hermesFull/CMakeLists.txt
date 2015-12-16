cmake_minimum_required(VERSION 2.8.3)
project(hermes_full)

# find all the required 'catkin' packages
find_package(catkin REQUIRED COMPONENTS
  hermes_core
)

# find required system packages
find_package(X11 REQUIRED)
sl_find_package_GLUT()
find_package(OpenGL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)

# find optional packages
find_package(xenomai_ros QUIET)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS hermes_core
)
sl_package()

if (Xenomai_FOUND)
  find_package(hermes_communication_tools REQUIRED)
  find_package(pf_imu REQUIRED)
  add_xenomai_flags()
endif()

include_directories(
  include
  math
  ${X11_INCLUDE_DIR}
  ${hermes_communication_tools_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Xenomai_INCLUDE_DIR}
  /home/arai/HERMES/src/cga_imu/include
  /home/arai/HERMES/src/cga_imu/src
  /home/arai/HERMES/src/pf_imu/include
  ${pf_imu_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

if (Xenomai_FOUND)
  include_directories( ${Xenomai_INCLUDE_DIR} )
endif()

# copy files from SL src to our src
# equivalent to the KeepUpToDateCopy macro in imake
sl_keep_up_to_date_copy(SL_dynamics.c ${sl_SOURCE_PREFIX}/src src)
sl_keep_up_to_date_copy(SL_forDynArt.c ${sl_SOURCE_PREFIX}/src src)
sl_keep_up_to_date_copy(SL_forDynComp.c ${sl_SOURCE_PREFIX}/src src)
sl_keep_up_to_date_copy(SL_invDynArt.c ${sl_SOURCE_PREFIX}/src src)
sl_keep_up_to_date_copy(SL_invDynNE.c ${sl_SOURCE_PREFIX}/src src)
sl_keep_up_to_date_copy(SL_kinematics.c ${sl_SOURCE_PREFIX}/src src)
sl_keep_up_to_date_copy(SL_parm_estimate.c ${sl_SOURCE_PREFIX}/src src)


# define libraries

# common objects
add_library(hermes_full
  src/SL_dynamics.c
  src/SL_forDynArt.cpp
  src/SL_forDynComp.cpp
  src/SL_invDynArt.cpp
  src/SL_invDynNE.cpp
  src/SL_kinematics.c
  ${hermes_core_SOURCE_PREFIX}/src/SL_user_commands.c
  src/SL_user_common.c
)
target_link_libraries(hermes_full
  SLcommon)

# openGL servo modules
add_library(hermes_full_openGL
  src/SL_user_openGL.c
)
target_link_libraries(hermes_full_openGL
  SLopenGL
  hermes_full
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${X11_LIBRARIES}
)

# task servo modules for simulation
add_library(hermes_full_task
  ${hermes_core_SOURCE_PREFIX}/src/SL_user_task.cpp
  ${hermes_core_SOURCE_PREFIX}/src/base_state_estimation.cpp
  ${hermes_core_SOURCE_PREFIX}/src/state_est_lin.cpp
  ${hermes_core_SOURCE_PREFIX}/src/COGDifferentiator.cpp
  ${hermes_core_SOURCE_PREFIX}/src/FallDetector.cpp
  ${hermes_core_SOURCE_PREFIX}/src/raw_data_access.cpp
)
target_link_libraries(hermes_full_task
  hermes_full
  SLtask
  cga_imu
  pf_imu
)

# simulation servo modules
add_library(hermes_full_simulation
  ${hermes_core_SOURCE_PREFIX}/src/SL_user_simulation.c
)
target_link_libraries(hermes_full_simulation
  hermes_full
  SLsimulation
)

# entry program for the hermes_full robot simulation
add_executable(xhermes_full
  ${hermes_core_SOURCE_PREFIX}/src/SL_main.c
  src/SL_user_common.c
)
target_link_libraries(xhermes_full
  ${X11_LIBRARIES}
  SLcommon
)
set_target_properties(xhermes_full
                      PROPERTIES OUTPUT_NAME xhermes)

# define executables

# parameter estimation
add_executable(xpest_hermes_full
  src/SL_parm_estimate.c
)
target_link_libraries(xpest_hermes_full
  hermes_full
  #SLcommon
)
set_target_properties(xpest_hermes_full
                      PROPERTIES OUTPUT_NAME xpest)

# motor servo modules for simulation
add_executable(xmotor_hermes_full
  ${hermes_core_SOURCE_PREFIX}/src/SL_user_motor.c
  ${hermes_core_SOURCE_PREFIX}/src/SL_user_sensor_proc_unix.c
)
target_link_libraries(xmotor_hermes_full
  hermes_full
  SLmotor
  #SLcommon
)
set_target_properties(xmotor_hermes_full
                      PROPERTIES OUTPUT_NAME xmotor)

#vision servo modules for simulation
add_executable(xvision_hermes_full
  ${hermes_core_SOURCE_PREFIX}/src/SL_user_vision.c
)
target_link_libraries(xvision_hermes_full
  hermes_full
  SLvision
  #SLcommon
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDM}
)
set_target_properties(xvision_hermes_full
		      PROPERTIES OUTPUT_NAME xvision)


# Now compile libs and programs for the real robot
site_name(HOSTNAME)
if (Xenomai_FOUND)
  message(STATUS "${PROJECT_NAME}: compiling on ${HOSTNAME}; including real robot.")

  # motor servo modules for the actual robot
  add_executable(xrmotor_hermes_full
    src/SL_user_common.c
    ${hermes_core_SOURCE_PREFIX}/src/valve_controller.cpp 
    ${hermes_core_SOURCE_PREFIX}/src/SL_user_sensor_proc_xeno.cpp 
    ${hermes_core_SOURCE_PREFIX}/src/gdc_sl_interface.cpp 
    ${hermes_core_SOURCE_PREFIX}/src/SL_user_motor_xeno.cpp
  )
  target_link_libraries(xrmotor_hermes_full
    hermes_full
    SLmotor
    ${Boost_LIBRARIES}
    hermes_communication_tools
    cga_imu
    pf_imu
  )
  set_target_properties(xrmotor_hermes_full
                        PROPERTIES OUTPUT_NAME xrmotor)

  # entry program for the actual robot
  add_executable(xrhermes_full
    ${hermes_core_SOURCE_PREFIX}/src/SL_rmain.c
    src/SL_user_common.c
  )
  target_link_libraries(xrhermes_full
    ${X11_LIBRARIES}
    SLcommon
  )
  set_target_properties(xrhermes_full
                        PROPERTIES OUTPUT_NAME xrhermes)
else()
  message(STATUS "${PROJECT_NAME}: compiling on ${HOSTNAME}; simulation only.")
endif()
