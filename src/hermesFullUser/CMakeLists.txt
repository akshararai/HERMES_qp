cmake_minimum_required(VERSION 2.8.3)
project(hermes_full_user)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_VERBOSE_MAKEFILE ON)

# find all required catkin packages
find_package(catkin REQUIRED COMPONENTS
  sl_config
  hermes_full
)

#find required system packages
find_package(X11 REQUIRED)
sl_find_package_GLUT()
find_package(OpenGL REQUIRED)
find_package(Boost COMPONENTS regex filesystem system thread REQUIRED)
find_package(Eigen REQUIRED)

# find optional packages
find_package(xenomai_ros QUIET)
find_package(biped_locomotion_lib QUIET)

#
# catkin package
#
catkin_package(
  CATKIN_DEPENDS hermes_full
)

sl_package()

if(Xenomai_FOUND)
  find_package(ati_ft_sensor REQUIRED)
  add_xenomai_flags()
endif()

include_directories(
  src/task/utils
  ${GLUT_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
  ${X11_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIRS}
  ${ati_ft_sensor_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}  
  ${biped_locomotion_lib_INCLUDE_DIRS}
)

if(Xenomai_FOUND)
  include_directories( ${Xenomai_INCLUDE_DIR} )
endif()


if(biped_locomotion_lib_FOUND)
  set(xtask_LINKED_LIBS ${xtask_LINKED_LIBS} biped_locomotion_lib_hermes_full)
  set(xopengl_LINKED_LIBS ${xopengl_LINKED_LIBS} biped_locomotion_lib_openGL_hermes_full)
  add_definitions(-DBIPED_LOCOMOTION_LIB)
endif()

#
# construct xtask
#
aux_source_directory(src/task/tasks xtask_FILES)
#aux_source_directory(src/task/utils xtask_FILES)

set(xtask_LINKED_LIBS ${xtask_LINKED_LIBS}
  SLtask hermes_full_task hermes_full 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# build xtask
add_executable(xtask_hermes_full
  ${xtask_FILES}
)
target_link_libraries(xtask_hermes_full ${xtask_LINKED_LIBS})
set_target_properties(xtask_hermes_full PROPERTIES OUTPUT_NAME xtask)

#
# construct xopengl
#
aux_source_directory(src/opengl xopengl_FILES)
set(xopengl_LINKED_LIBS ${xopengl_LINKED_LIBS}
  hermes_full hermes_full_openGL
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${X11_LIBRARIES})

# build xopengl
add_executable(xopengl_hermes_full
  ${xopengl_FILES}
)
target_link_libraries(xopengl_hermes_full 
  ${xopengl_LINKED_LIBS}
)
set_target_properties(xopengl_hermes_full
                      PROPERTIES OUTPUT_NAME xopengl)

add_executable(xsimulation_hermes_full
  src/sim/initUserSimulation.c
)
target_link_libraries(xsimulation_hermes_full hermes_full hermes_full_simulation)
set_target_properties(xsimulation_hermes_full
                      PROPERTIES OUTPUT_NAME xsimulation)

sl_install_binary(${hermes_full_SOURCE_PREFIX}/${MACHTYPE} xhermes xhermes_full)
sl_install_binary(${hermes_full_SOURCE_PREFIX}/${MACHTYPE} xmotor xmotor_hermes_full)
sl_install_binary(${hermes_full_SOURCE_PREFIX}/${MACHTYPE} xvision xvision_hermes_full)

site_name(HOSTNAME)
if (Xenomai_FOUND)
  sl_install_binary(${hermes_full_SOURCE_PREFIX}/${MACHTYPE} xrhermes xrhermes_full)
  sl_install_binary(${hermes_full_SOURCE_PREFIX}/${MACHTYPE} xrmotor xrmotor_hermes_full)

  set(xtask_FILES ${xtask_FILES} src/task/tasks/ati_sensor.cpp)

  add_executable(xrtask_hermes_full
    ${xtask_FILES}
  )
  target_link_libraries(xrtask_hermes_full ${xtask_LINKED_LIBS} ati_ft_sensor)
  set_target_properties(xrtask_hermes_full PROPERTIES COMPILE_DEFINITIONS "USE_ATI_SENSOR"  OUTPUT_NAME xrtask)
endif()
