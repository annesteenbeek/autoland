cmake_minimum_required(VERSION 2.8.3)
project(autoland)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  #  mavros
  #  mavros_extras
  roscpp
  pid
  rospy
  message_generation
)
## Generate services in the 'srv' folder
 add_service_files(
    FILES
    StartAutoland.srv
)

generate_messages(
    DEPENDENCIES 
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autoland
#  CATKIN_DEPENDS geometry_msgs mavros roscpp
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(autoland
#   src/${PROJECT_NAME}/autoland.cpp
# )


