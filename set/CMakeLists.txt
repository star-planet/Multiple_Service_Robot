cmake_minimum_required(VERSION 2.8.3)
project(set)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  std_srvs
  tf
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  turtlebot3_msgs
  message_generation
  message_runtime
  nav_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

####################
## RPi GPIO setup ##
####################
# set (wiringPi_include "/use/local/include")

find_package(JetsonGPIO)

catkin_python_setup()

add_service_files(
  FILES
  setlocation.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   std_srvs
   geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES
    set
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    std_srvs
    geometry_msgs
    actionlib_msgs
    move_base_msgs
    turtlebot3_msgs
    message_generation
    message_runtime
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(setpose src/setpose.cpp)
# target_link_libraries(setpose ${catkin_LIBRARIES})

# add_executable(ultrasonic src/ultrasonic.cpp)
# target_link_libraries(ultrasonic JetsonGPIO::JetsonGPIO ${catkin_LIBRARIES})

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  nodes/rotate
  nodes/firebase
  nodes/loadmap
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############