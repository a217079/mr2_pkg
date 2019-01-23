cmake_minimum_required(VERSION 2.8.3)
project(mr2_driver)

add_compile_options(-std=c++11)

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

add_executable(mr2_simple_rotation_node src/simple_rotation.cpp src/ik_solver.cpp src/transformation.cpp)
target_link_libraries(mr2_simple_rotation_node ${catkin_LIBRARIES})
add_dependencies(mr2_simple_rotation_node mr2_driver_generate_messages_cpp)

add_executable(mr2_sand_dune_node src/sand_dune.cpp src/ik_solver.cpp src/transformation.cpp)
target_link_libraries(mr2_sand_dune_node ${catkin_LIBRARIES})
add_dependencies(mr2_sand_dune_node mr2_driver_generate_messages_cpp)

add_executable(mr2_normal_gait_node src/normal_gait.cpp src/ik_solver.cpp src/transformation.cpp)
target_link_libraries(mr2_normal_gait_node ${catkin_LIBRARIES})
add_dependencies(mr2_normal_gait_node mr2_driver_generate_messages_cpp)

add_executable(mr2_tussock_gait_node src/tussock_gait.cpp src/ik_solver.cpp src/transformation.cpp)
target_link_libraries(mr2_tussock_gait_node ${catkin_LIBRARIES})
add_dependencies(mr2_tussock_gait_node mr2_driver_generate_messages_cpp)

add_executable(mr2_csv_node src/csv_gait.cpp src/ik_solver.cpp src/transformation.cpp)
target_link_libraries(mr2_csv_node ${catkin_LIBRARIES})
add_dependencies(mr2_csv_node mr2_driver_generate_messages_cpp)
