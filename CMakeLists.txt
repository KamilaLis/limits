project(limits)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

##############
## Packages ##
##############

## Catkin ##

find_package(catkin REQUIRED COMPONENTS
  roscpp
  manager_api
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## Locations ##

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Executables ##

add_executable(limits_node
  src/limits_node.cpp
)

target_link_libraries(limits_node
  ${catkin_LIBRARIES}
)

