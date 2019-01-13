cmake_minimum_required(VERSION 2.8.3)
project(mr2_driver)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  message_generation
  cmake_modules
)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  LegsCommands.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mr2_simple_creep_node src/simple_creep_gait.cpp src/ik_solver.cpp src/transformation.cpp)
target_link_libraries(mr2_simple_creep_node ${catkin_LIBRARIES})
add_dependencies(mr2_simple_creep_node mr2_driver_generate_messages_cpp)

add_executable(mr2_command_publisher_node src/command_publisher.cpp)
target_link_libraries(mr2_command_publisher_node ${catkin_LIBRARIES})
add_dependencies(mr2_command_publisher_node mr2_driver_generate_messages_cpp)
