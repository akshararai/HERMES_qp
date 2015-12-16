# check if MACHTYPE environment variable exists
if (DEFINED ENV{MACHTYPE})
  #message(STATUS "Compiling for MACHTYPE=$ENV{MACHTYPE}")
  set(MACHTYPE $ENV{MACHTYPE})
else()
  #  message(FATAL_ERROR "Couldn't find the MACHTYPE environment variable! Please source the SL-CONFIG.sh script from the sl_config package.")
  exec_program(${CMAKE_CURRENT_LIST_DIR}/../scripts/get_machtype.bash
    OUTPUT_VARIABLE MACHTYPE
  )
message(STATUS "MACHTYPE=${MACHTYPE}")
endif()

# include macros
include(${CMAKE_CURRENT_LIST_DIR}/sl_macros.cmake)

# include our own cmake modules, with higher priority
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_MODULE_PATH})

# set robot machine names globally
set(ARM_HOST mandy)
set(APOLLO_HOST pechstein)
set(HERMES_HOST hermes)

# set global compile type
set(CMAKE_BUILD_TYPE RelWithDebInfo) # Optimization with debugging info
#set(CMAKE_BUILD_TYPE Release)       # Optimization 
#set(CMAKE_BUILD_TYPE Debug)         # Debug

# set global compile flags
add_definitions(-DUNIX)
add_definitions(-Wall -Wno-unused -Wno-strict-aliasing)
add_definitions(-D${MACHTYPE})

# set custom flags per architecture
if (APPLE) # apple, mac os x

  # allow nested functions on mac; linux gcc doesn't like this flag
  #  add_definitions(-fnested-functions)

  # SL libraries are built with undefined symbols, some compilers don't like that
  set(SL_IGNORE_UNDEF_SYMBOLS "-flat_namespace -undefined dynamic_lookup")

else() # only linux so far, with or without xenomai

  # SL libraries are built with undefined symbols, some compilers don't like that
  set(SL_IGNORE_UNDEF_SYMBOLS " ")

  # remove "as-needed" linking for binaries; only on linux
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")

endif()
