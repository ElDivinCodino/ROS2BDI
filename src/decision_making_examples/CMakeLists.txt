cmake_minimum_required(VERSION 3.5)
project(decision_making_examples)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()



###########
## Build ##
###########
add_executable(fsm_roomba src/fsm/Roomba.cpp)
add_executable(fsm_roomba_events src/fsm/RoombaEvents.cpp)
add_executable(fsm_turnstile src/fsm/Turnstile.cpp)
add_executable(fsm_turnstile_events src/fsm/TurnstileEvents.cpp)
add_executable(fsm_wandering src/fsm/Wandering.cpp)
add_executable(fsm_wandering_events src/fsm/WanderingEvents.cpp)
add_executable(hsm_ex src/hsm/ScxmlHierarchyExample.cpp)
#add_executable(bt_ex src/bt/Example1.cpp)
add_executable(tao_ex src/tao/TaoExample.cpp)
add_executable(tao_increment src/tao/TaoIncrement.cpp)
add_executable(hsm_mission src/hsm/Mission.cpp)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(decision_making REQUIRED)
find_package(std_msgs REQUIRED)
find_package(decision_making_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(random_numbers REQUIRED)
find_package(rclcpp REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
  ${decision_making_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${decision_making_parser_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS}
  ${random_numbers_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${decision_making_LIBRARIES} ${std_msgs_LIBRARIES}
  ${decision_making_parser_LIBRARIES} ${sensor_msgs_LIBRARIES}
  ${random_numbers_LIBRARIES} ${rclcpp_LIBRARIES})
target_link_libraries(fsm_roomba ${LIBRARY_DIRS})
target_link_libraries(fsm_roomba_events ${LIBRARY_DIRS})
target_link_libraries(fsm_turnstile ${LIBRARY_DIRS})
target_link_libraries(fsm_turnstile_events ${LIBRARY_DIRS})
target_link_libraries(fsm_wandering ${LIBRARY_DIRS})
target_link_libraries(fsm_wandering_events ${LIBRARY_DIRS})
target_link_libraries(hsm_ex ${LIBRARY_DIRS})
target_link_libraries(tao_ex ${LIBRARY_DIRS})
target_link_libraries(tao_increment ${LIBRARY_DIRS})
target_link_libraries(hsm_mission ${LIBRARY_DIRS})
set(LIBS ${ament_cmake_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${decision_making_LIBRARIES} ${std_msgs_LIBRARIES}
  ${decision_making_parser_LIBRARIES} ${sensor_msgs_LIBRARIES}
  ${random_numbers_LIBRARIES} ${rclcpp_LIBRARIES})
#target_link_libraries(bt_ex ${catkin_LIBRARIES})
decision_making_parsing(src/fsm/Roomba.cpp src/fsm/Turnstile.cpp
  src/fsm/Wandering.cpp src/hsm/ScxmlHierarchyExample.cpp  #	src/bt/Example1.cpp
  src/tao/TaoExample.cpp src/tao/TaoIncrement.cpp src/hsm/Mission.cpp)

#############
## Install ##
#############

install(TARGETS fsm_roomba fsm_roomba_events fsm_turnstile fsm_turnstile_events
  fsm_wandering fsm_wandering_events
  RUNTIME
  DESTINATION bin)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(decision_making)
ament_export_dependencies(std_msgs)
ament_export_dependencies(decision_making_parser)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(random_numbers)
ament_export_dependencies(rclcpp)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
