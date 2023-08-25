cmake_minimum_required(VERSION 3.0.2)
project(frenet)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  tf2_ros
  message_filters
  tf2_geometry_msgs
  actionlib_msgs
  nav_msgs
  obstacle_detector
)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

add_action_files(
  DIRECTORY action
  FILES Frenet.action
)

# add_message_files(DIRECTORY msg FILES frenet.msg)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES detecting_wall
  CATKIN_DEPENDS roscpp actionlib_msgs message_runtime
  DEPENDS Eigen
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
  include/${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DIRS}
)

add_library(QuinticPolynomial ./src/QuinticPolynomial.cpp ./include/frenet/QuinticPolynomial.hpp)
add_library(CublicSpline ./src/CublicSpline.cpp ./include/frenet/CublicSpline.hpp)
set(FrenetPath ./include/frenet/FrenetPath.hpp)

add_executable(mode_one src/mode_one.cpp ${FrenetPath})
target_link_libraries(mode_one CublicSpline QuinticPolynomial ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(mode_one ${catkin_EXPORTED_TARGETS})

# add_executable(mode_one src/mode_one.cpp ${FrenetPath})
# target_link_libraries(mode_one QuinticPolynomial CublicSpline ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
# add_dependencies(mode_one ${catkin_EXPORTED_TARGETS})
