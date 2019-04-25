cmake_minimum_required(VERSION 3.5)
project(bdi_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include 
            ${rclcpp_INCLUDE_DIRS}
            ${rmw_implementation_INCLUDE_DIRS}
            ${std_msgs_INCLUDE_DIRS}
)

add_library(custom_objects
            beliefs/belief.cpp
            goals/goal.cpp
            plans/plan.cpp
            plans/cleaningRoutine.cpp
            plans/cleaningSpecific.cpp
            plans/rechargeBatteryCritical.cpp
            plans/rechargeBatterySafe.cpp
)

target_link_libraries(custom_objects ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${std_msgs_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


set(msg_files
  "msg/BeliefBool.msg"
  "msg/BeliefString.msg"
  "msg/BeliefInt.msg"
  "msg/BeliefFloat.msg"
  "msg/GoalBool.msg"
  "msg/GoalString.msg"
  "msg/GoalInt.msg"
  "msg/GoalFloat.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

add_executable(belief_node src/belief_node.cpp)
ament_target_dependencies(belief_node rclcpp std_msgs)
rosidl_target_interfaces(belief_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(goal_node src/goal_node.cpp)
ament_target_dependencies(goal_node rclcpp std_msgs)
rosidl_target_interfaces(goal_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(scheduler_node src/scheduler_node.cpp)
ament_target_dependencies(scheduler_node rclcpp std_msgs)
rosidl_target_interfaces(scheduler_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(interpreter_node src/interpreter_node.cpp)
ament_target_dependencies(interpreter_node rclcpp std_msgs)
rosidl_target_interfaces(interpreter_node ${PROJECT_NAME} "rosidl_typesupport_cpp")




add_executable(batterySensor src/RobotVacuum/batterySensor.cpp)
ament_target_dependencies(batterySensor rclcpp std_msgs)
rosidl_target_interfaces(batterySensor ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(tankSensor src/RobotVacuum/tankSensor.cpp)
ament_target_dependencies(tankSensor rclcpp std_msgs)
rosidl_target_interfaces(tankSensor ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(internalStateMonitor src/RobotVacuum/internalStateMonitor.cpp)
ament_target_dependencies(internalStateMonitor rclcpp std_msgs)
rosidl_target_interfaces(internalStateMonitor ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(goalManager src/RobotVacuum/goalManager.cpp)
ament_target_dependencies(goalManager rclcpp std_msgs)
rosidl_target_interfaces(goalManager ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(scheduler src/RobotVacuum/scheduler.cpp)
target_link_libraries(scheduler custom_objects)
ament_target_dependencies(scheduler rclcpp std_msgs)
rosidl_target_interfaces(scheduler ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(interpreter src/RobotVacuum/interpreter.cpp)
target_link_libraries(interpreter custom_objects)
ament_target_dependencies(interpreter rclcpp std_msgs)
rosidl_target_interfaces(interpreter ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(DIRECTORY
    include/${PROJECT_NAME}
    DESTINATION include
)

install(TARGETS
   belief_node
   goal_node
   scheduler_node
   interpreter_node

   custom_objects

   batterySensor
   tankSensor
   internalStateMonitor
   goalManager
   scheduler
   interpreter

   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()