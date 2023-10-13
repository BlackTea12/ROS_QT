cmake_minimum_required(VERSION 3.0.2)
project(qt_ros)

find_package(catkin REQUIRED COMPONENTS 
   rospy
)

catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  ${catkin_INCLUDE_DIRS}
)


