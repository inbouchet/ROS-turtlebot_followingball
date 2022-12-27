cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_ball_following)

find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  rospy
  roscpp
  std_msgs
  message_generation
  gazebo_ros
)

add_message_files(
  FILES
  ImageInfo.msg
  BallMoveInfo.msg
)

add_service_files(
  FILES
  Attach.srv
  ManipulationCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS
   scripts/video_capture.py
   scripts/tracking_command.py
   scripts/manipulation_command.py
   scripts/tests/ball_tracking_v0.py
   scripts/tests/image_processing/check_image_info.py
   scripts/tests/image_processing/get_image_process.py
   scripts/tests/pick/attach.py
   scripts/tests/pick/detach.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(gazebo REQUIRED)

include_directories(include)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME} plugins/gazebo_ros_link_attacher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(ball_moving_plugin SHARED plugins/ball_moving_plugin.cc)
target_link_libraries(ball_moving_plugin ${GAZEBO_LIBRARIES})