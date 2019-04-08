cmake_minimum_required(VERSION 3.5)
project(decision_making)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#find_package(robot_task REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(decision_making
#   src/${PROJECT_NAME}/decision_making.cpp
# )

## Declare a cpp executable

add_executable(bt_example src/BTExample.cpp)


add_executable(tao_example src/TAOExample.cpp)


#add_executable(mult_hyb_example src/multifiles/HybridExample.cpp)
#target_link_libraries(mult_hyb_example ${catkin_LIBRARIES} )


#===================== ROS =============================================

add_library(decision_making_ros src/ROSTask.cpp)
add_dependencies(decision_making_ros robot_task_gencpp)

add_executable(ros_example src/RosExample.cpp)
find_package(ament_cmake REQUIRED)
find_package(decision_making_parser REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(robot_task REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS}
  ${decision_making_parser_INCLUDE_DIRS} ${diagnostic_updater_INCLUDE_DIRS}
  ${robot_task_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${decision_making_parser_LIBRARIES}
  ${diagnostic_updater_LIBRARIES} ${robot_task_LIBRARIES} ${std_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES})
target_link_libraries(bt_example ${LIBRARY_DIRS})
target_link_libraries(tao_example ${LIBRARY_DIRS})
target_link_libraries(ros_example ${LIBRARY_DIRS})
set(LIBS ${ament_cmake_LIBRARIES} ${decision_making_parser_LIBRARIES}
  ${diagnostic_updater_LIBRARIES} ${robot_task_LIBRARIES} ${std_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES})

#=======================================================================

#add_executable(fsm_example src/FSMExample.cpp)
#target_link_libraries(fsm_example ${catkin_LIBRARIES} )


#add_executable(task_test src/ROSTask.cpp)
#target_link_libraries(task_test ${catkin_LIBRARIES} )


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(decision_making_node decision_making_generate_messages_cpp)

## Specify libraries to link a library or executable target against

#==================== PARSER ===========================================

#add_executable(parse_code src/test_parser.cpp)
#message("do decision_making_parsing")


decision_making_parsing(src/RosExample.cpp src/BTExample.cpp src/ROSTask.cpp
  src/FSMExample.cpp src/HybridExample.cpp) #=======================================================================
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION bin
# )

## Mark executables and/or libraries for installation
# install(TARGETS decision_making decision_making_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_decision_making.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(decision_making_parser)
ament_export_dependencies(diagnostic_updater)
ament_export_dependencies(robot_task)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rclcpp)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(decision_making_ros ${LIBS})

ament_package()
