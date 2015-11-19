cmake_minimum_required(VERSION 2.8.3)
project(utilities)

find_package(catkin REQUIRED COMPONENTS sl_config)

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
  LIBRARIES utility
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
sl_package()

include_directories(include)

add_library(utility
    src/balanc.c
    src/brent.c
    src/choldc.c
    src/control.c
    src/data_sets.c
    src/dbrent.c
    src/df1dim.c
    src/dfpmin.c
    src/distributions.c
    src/dlinmin.c
    src/elmhes.c
    src/f1dim.c
    src/frprmn.c
    src/hqr.c
    src/kdtree.c
    src/j-fixed.c
    src/ldltinv.c
    src/linmin.c
    src/lookup_table.c
    src/lubksb.c
    src/ludcmp.c
    src/mnbrak.c
    src/numerics_package.c
    src/parm_opt.c
    src/powell.c
    src/pythag.c
    src/stability.c
    src/statistics.c
    src/svbksb.c
    src/svdcmp.c
    src/utility.c
)
target_link_libraries(utility m)

#install(TARGETS utility
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(FILES
#  include/utility.h
#  include/data_sets.h
#  include/nrutil.h
#  include/statistics.h
#  include/utility_macros.h
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

