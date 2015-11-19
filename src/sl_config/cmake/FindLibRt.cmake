# Find librt (only on Linux)
# Defines LibRt_LIBRARIES

if (UNIX AND NOT APPLE)
  find_library(LibRt_LIBRARY rt)
  set(LibRt_LIBRARIES ${LibRt_LIBRARY})
  include (FindPackageHandleStandardArgs)
  find_package_handle_standard_args(LibRt DEFAULT_MSG LibRt_LIBRARIES)
else()
  set(LibRt_LIBRARIES "")
endif()

mark_as_advanced(LibRt_LIBRARIES)
