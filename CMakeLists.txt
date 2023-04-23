cmake_minimum_required(VERSION 3.5)
project(plansys2_gpsr_tayros2)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(nav2_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_bt_actions
    nav2_msgs
)

include_directories(include)

add_executable(close_door_action_node src/close_door_action_node.cpp)
ament_target_dependencies(close_door_action_node ${dependencies})

add_executable(drop_action_node src/drop_action_node.cpp)
ament_target_dependencies(drop_action_node ${dependencies})

add_executable(pick_prio_action_node src/pick_prio_action_node.cpp)
ament_target_dependencies(pick_prio_action_node ${dependencies})

add_executable(gpsr_controller_node src/gpsr_controller_node.cpp)
ament_target_dependencies(gpsr_controller_node ${dependencies})

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_library(plansys2_closedoor_bt_node SHARED src/behavior_tree_nodes/CloseDoor.cpp)
list(APPEND plugin_libs plansys2_closedoor_bt_node)

add_library(plansys2_closegripper_bt_node SHARED src/behavior_tree_nodes/CloseGripper.cpp)
list(APPEND plugin_libs plansys2_closegripper_bt_node)

add_library(plansys2_opengripper_bt_node SHARED src/behavior_tree_nodes/OpenGripper.cpp)
list(APPEND plugin_libs plansys2_opengripper_bt_node)

add_library(plansys2_movebydoor_bt_node SHARED src/behavior_tree_nodes/MoveByDoor.cpp)
list(APPEND plugin_libs plansys2_movebydoor_bt_node)

add_library(plansys2_opendoor_bt_node SHARED src/behavior_tree_nodes/OpenDoor.cpp)
list(APPEND plugin_libs plansys2_opendoor_bt_node)

add_library(plansys2_pick_bt_node SHARED src/behavior_tree_nodes/Pick.cpp)
list(APPEND plugin_libs plansys2_pick_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(DIRECTORY launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  close_door_action_node
  pick_prio_action_node
  drop_action_node
  gpsr_controller_node
  move_action_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()

