export LAB_ROOT="@(CMAKE_CURRENT_SOURCE_DIR)/.."
export PROG_ROOT=$LAB_ROOT
export HOST=`hostname` 
export MACHTYPE=`@(CMAKE_CURRENT_SOURCE_DIR)/scripts/get_machtype.bash`
export PATH=./$MACHTYPE:$PATH 
