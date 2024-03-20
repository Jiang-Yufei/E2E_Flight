cmake_minimum_required(VERSION 3.0.2)
project(data_collection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES data_collection
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge
#  DEPENDS system_lib
)


include_directories(
    SYSTEM
    include
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(data_collection_node
	src/data_collection_node.cpp
	src/data_collection_manager.cpp
	)


## Specify libraries to link a library or executable target against
target_link_libraries(data_collection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
