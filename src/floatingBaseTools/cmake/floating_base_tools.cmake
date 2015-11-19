# disable eigen stack size limits
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)

set(floating_base_tools_DEFINITIONS "")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
else()
  set(floating_base_tools_DEFINITIONS "${floating_base_tools_DEFINITIONS};EIGEN_NO_DEBUG;RTEIG_NO_ASSERTS")
endif()

# py_cpp
find_package(py_cpp_interface QUIET)
find_package(PythonLibs QUIET)
if(py_cpp_interface_FOUND AND PYTHONLIBS_FOUND)
#  list(APPEND floating_base_tools_DEFINITIONS "py_cpp_interface_EXISTS")
  set(py_cpp_interface_REQ_SATISFIED TRUE)
else()
  set(py_cpp_interface_REQ_SATISFIED FALSE)
endif() 

# robots that are available
set(floating_base_tools_ROBOTS)
FOREACH (ROB hermes_lower hermes_full athena)
  find_package(${ROB} QUIET)
  if(${${ROB}_FOUND})
    list(APPEND floating_base_tools_ROBOTS ${ROB})
  endif()  
ENDFOREACH(ROB)


FOREACH (ROB ${floating_base_tools_ROBOTS})
SET(floating_base_tools_${ROB} ${floating_base_tools_${ROB}} ${py_cpp_interface_LIBRARIES})
ENDFOREACH(ROB)
