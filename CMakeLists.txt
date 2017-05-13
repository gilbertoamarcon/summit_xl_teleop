cmake_minimum_required(VERSION 2.8.3)
project(summit_xl_teleop)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    diagnostic_updater
    )
    
add_service_files(DIRECTORY srv 
   FILES
    enable_disable_pad.srv
)

generate_messages(
   DEPENDENCIES
   robotnik_msgs sensor_msgs summit_xl_teleop
 )

catkin_package(
  CATKIN_DEPENDS 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    message_runtime
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

add_executable(summit_xl_teleop src/summit_xl_teleop.cpp)

add_dependencies(summit_xl_teleop summit_xl_teleop_generate_messages_cpp)

target_link_libraries(summit_xl_teleop
   ${catkin_LIBRARIES}
 )

install(
  TARGETS summit_xl_teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
