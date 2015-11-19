cmake_minimum_required(VERSION 2.8.3)
project(sl_config)

find_package(catkin REQUIRED)

catkin_add_env_hooks(20.sl_config SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

include(cmake/sl_config.cmake)

catkin_package(
  CFG_EXTRAS sl_config.cmake
)

