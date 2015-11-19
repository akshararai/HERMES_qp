cmake_minimum_required(VERSION 2.8.3)
project(lwpr)

find_package(catkin REQUIRED COMPONENTS
  sl_config utilities)

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
  LIBRARIES lwpr
  CATKIN_DEPENDS utilities
#  DEPENDS system_lib
)
sl_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(lwpr
  src/lwpr.c
  src/lwpr_script.c
)
target_link_libraries(lwpr ${utilities_LIBRARIES})

add_executable(xascii2mrplot
  src/ascii2mrdplot.c
)
target_link_libraries(xascii2mrplot lwpr)

add_executable(xlwpr
  src/lwpr_test.c
  src/lwpr_main.c
)
target_link_libraries(xlwpr lwpr)

add_executable(xlwprstat
  src/lwpr_stat.c
  src/lwpr_main.c
)
target_link_libraries(xlwprstat lwpr)

#install(TARGETS lwpr xascii2mrplot xlwpr xlwprstat
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES include/lwpr.h
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
