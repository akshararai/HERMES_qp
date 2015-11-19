cmake_minimum_required(VERSION 2.8.3)
project(floating_base_tools)

set(BUILD_DOCUMENTATION OFF)
#set(CMAKE_BUILD_TYPE debug)

# find SL dependencies using catkin
find_package(catkin REQUIRED COMPONENTS sl)

# required packages
find_package(Boost REQUIRED COMPONENTS thread system filesystem regex)
find_package(Eigen REQUIRED)

include(cmake/floating_base_tools.cmake)

# find optional packages
# ros
find_package(xenomai_ros QUIET)
find_package(roscpp QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
if(roscpp_FOUND AND sensor_msgs_FOUND AND geometry_msgs_FOUND)
  set(ros_REQ_SATISFIED True)
else()
  set(ros_REQ_SATISFIED False)
endif()
message("ros_REQ_SATISFIED: ${ros_REQ_SATISFIED}")

catkin_package(
  INCLUDE_DIRS include ${py_cpp_interface_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS}
  CATKIN_DEPENDS sl
  CFG_EXTRAS floating_base_tools.cmake
)
sl_package()

if (Xenomai_FOUND)
  add_xenomai_flags()
endif()

include_directories(
  include  
  ${sl_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  # optional packages
  ${Xenomai_INCLUDE_DIR}
  ${py_cpp_interface_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

#
# floating_base_tools
#
set(floating_base_tools_SRC_FILES
  src/RtMatrixX.cpp
  src/CartesianPDCmd.cpp
  src/ConfigUtils.cpp
  src/ContactHelper.cpp
  src/EigenQpSolver.cpp
  src/FileSequence.cpp
  src/FloatingBaseKinematics.cpp
  src/GeometryUtils.cpp
  src/KinematicsEigen.cpp
  src/MomentumComputation.cpp
  src/OptimalInverseDynamicsEigen.cpp
  src/floatingBaseToolsRobotInfo.cpp
)
set(floating_base_tools_LIBS
  SLtask
  ${Boost_LIBRARIES}
  #optional packages
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDM}
  ${py_cpp_interface_LIBRARIES}
)
# add optional components
if(py_cpp_interface_REQ_SATISFIED)
  set(floating_base_tools_LIBS ${floating_base_tools_LIBS}
    ${PYTHON_LIBRARIES})
endif()
if(ros_REQ_SATISFIED)
  #set(floating_base_tools_SRC_FILES ${floating_base_tools_SRC_FILES}
  #  src/sl_tasks/dbg_contact_helper.cpp)
  include_directories(
    ${roscpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
  )
  set(floating_base_tools_LIBS ${floating_base_tools_LIBS}
    ${roscpp_LIBRARIES}
)
endif()

FOREACH(ROB ${floating_base_tools_ROBOTS})
    message(STATUS "building floating_base_tools_${ROB}")
    add_library(floating_base_tools_${ROB} ${floating_base_tools_SRC_FILES})
    target_link_libraries(floating_base_tools_${ROB} ${floating_base_tools_LIBS} ${ROB}) 
    set_target_properties(floating_base_tools_${ROB} PROPERTIES COMPILE_FLAGS "-I${${ROB}_SOURCE_PREFIX}/include")
    
    add_executable(solve_lexmin_${ROB} EXCLUDE_FROM_ALL
      tests/solve_lexmin.cpp
    )
    target_link_libraries(solve_lexmin_${ROB}
    floating_base_tools_${ROB}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_REGEX_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_LIBRARIES})
ENDFOREACH(ROB)


#
# py_cpp
#
#LIST(GET floating_base_tools_ROBOTS 0 SOME_ROBOT)
#catkin_python_setup()
#if(py_cpp_interface_REQ_SATISFIED)
#  add_executable(test_CvxopQP
#    tests/test_CvxopQp.cpp
#    src/ConfigUtils.cpp
#    src/FileSequence.cpp
#  )
#  target_link_libraries(test_CvxopQP
#    ${Boost_LIBRARIES}
#    floating_base_tools_${SOME_ROBOT}
#    #optional packages
#    ${py_cpp_interface_LIBRARIES}
#    ${PYTHON_LIBRARIES}
#  )
#endif()


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