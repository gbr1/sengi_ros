# The MIT License
# 
# Copyright (c) 2019 Giovanni di Dio Bruno https://gbr1.github.io
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#
cmake_minimum_required(VERSION 2.8.3)
project(sengi_ros)

find_package(catkin REQUIRED COMPONENTS 
              roscpp
              message_generation
              std_msgs
              controller_manager
              hardware_interface
              realtime_tools
              topic_tools
              sensor_msgs
              roslaunch
              rosserial_server
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES
  PidConfiguration.msg
  JointFeedback.msg
  Pid.msg
  Status.msg
  Drive.msg
  Feedback.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES sengi_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)


add_executable(sengi_node
  src/sengi_node.cpp
)
add_dependencies(sengi_node sengi_ros_gencpp)
target_link_libraries(sengi_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include )

install(DIRECTORY src/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src )

install(DIRECTORY msg/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg )