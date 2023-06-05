cmake_minimum_required(VERSION 3.0.2)
project(gndlib)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++14)

################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
  roscpp
  std_msgs
  )

set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5PrintSupport
  Qt5Core
  Qt5Gui
  gnd_msgs
  #OpenCV
  )

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::PrintSupport
  Qt5::Core
  Qt5::Gui
  #${OpenCV_LIBS}
  )

set(INCLUDE_DIRECTORIES
  #${OpenCV_LIBS}
  )


##########################################
##  Be Carefull editing below this line ##
##########################################

## Compile as C++14
add_compile_options(-std=c++14)

## need this for qt auto generated headers to include!
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###
###  Add Non ROS dependencies
###

foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

###
###  Add any required ros packages here
###
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})

catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES
#    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${ROS_DEPENDS}
  DEPENDS
    ${EXTERNAL_DEPENDS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRECTORIES}
)

###########
## Build ##
###########

###
### QT stuff (required if you want a GUI)
###
set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files


#install(TARGETS ${NODE_LIST}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  )

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN "*.hpp"
#  PATTERN ".svn" EXCLUDE
#)
