cmake_minimum_required(VERSION 2.8.3)
project(loam_back_and_forth)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(scanRegistration src/scanRegistration.cpp)
add_executable(laserOdometry src/laserOdometry.cpp)
add_executable(laserMapping src/laserMapping.cpp)
add_executable(transformMaintenance src/transformMaintenance.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES})
