cmake_minimum_required(VERSION 2.8.3)
project(hermes_core)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sl_config sl
)

INCLUDE_DIRECTORIES(
  home/arai/HERMES/src/cga_imu/include
  ${pf_imu_INCLUDE_DIRS} )

