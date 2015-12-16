# convenience macros for use in SL cmake files

# sets up standard binary output directories for sl packages
# call this after catkin_package()
macro(sl_package)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${MACHTYPE})
endmacro()

# creates a make target that copies fileName form source to target
macro(sl_keep_up_to_date_copy fileName sourceDirectory targetDirectory)
  set (srcFile ${sourceDirectory}/${fileName})
  set (destFile ${targetDirectory}/${fileName})
  set (targetName ${destFile})
  add_custom_command(
    OUTPUT ${targetName}
    COMMAND ${CMAKE_COMMAND} -E copy ${srcFile} ${destFile}
    DEPENDS ${srcFile}
  )
endmacro()

#find GLUT libraries s.t. it is compatible with mac
macro(sl_find_package_GLUT)
  if("${MACHTYPE}" STREQUAL "x86_64mac")
    set(GLUT_LIBRARIES "/opt/X11/lib/libglut.dylib")
    set(GLUT_INCLUDE_DIR /opt/X11/include)
    if(EXISTS "${GLUT_LIBRARIES}" AND EXISTS "${GLUT_INCLUDE_DIR}" AND IS_DIRECTORY "${GLUT_INCLUDE_DIR}")
      set(GLUT_FOUND TRUE)
    else()
      set(GLUT_FOUND FALSE)
      set(GLUT_LIBRARIES)
      set(GLUT_INCLUDE_DIR)
    endif()
  else()
    find_package(GLUT REQUIRED)
  endif()
endmacro()


# sl_install_binary: installs a file to our local binary directory
#
# the file is copied from sourceDirectory/fileName to fileName in the target directory
#
# an optional argument "dependencyName" can be passed if the cmake target that created
# the original file was somehow named differently
macro(sl_install_binary sourceDirectory fileName)
  set (srcFile ${sourceDirectory}/${fileName})
  set (destFile ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${fileName})
  set (targetName ${destFile})
  set (dependencyName ${srcFile})
  set (customTargetName ${fileName})
  foreach(extraArg ${ARGN})
    set (dependencyName ${extraArg})
    set (customTargetName ${extraArg})
  endforeach()
  add_custom_command(
    OUTPUT ${targetName}
    COMMAND ${CMAKE_COMMAND} -E copy ${srcFile} ${destFile}
    DEPENDS ${dependencyName}
  )
  add_custom_target(
    install_machtype_${customTargetName} ALL
    DEPENDS ${targetName}
  )
endmacro()
