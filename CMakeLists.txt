cmake_minimum_required(VERSION 3.5)
project(naosoccer_pos_action)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  rosidl_default_generators
  ament_index_cpp
  Boost
  nao_lola_sensor_msgs
  nao_lola_command_msgs
  std_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "action/Pos.action"
)

# Build
add_library(${PROJECT_NAME}_node SHARED
  src/naosoccer_pos_action_node.cpp
  src/parser.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "naosoccer_pos_action::NaosoccerPosActionNode"
  EXECUTABLE ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(
  DIRECTORY pos/
  DESTINATION share/${PROJECT_NAME}/pos/
)

# Actions
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(naosoccer_pos_action_node "${cpp_typesupport_target}")

add_library(action_server SHARED
        src/naosoccer_pos_action_server.cpp)
target_include_directories(action_server PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
        PRIVATE "NAOSOCCER_POS_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
        "rclcpp"
        "rclcpp_action"
        "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "naosoccer_pos_action::NaosoccerPosActionServer" EXECUTABLE naosoccer_pos_action_server)
install(TARGETS
        action_server
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

add_library(action_client SHARED
        src/naosoccer_pos_action_client.cpp)
target_include_directories(action_client PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
        PRIVATE "NAOSOCCER_POS_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
        "rclcpp"
        "rclcpp_action"
        "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "naosoccer_pos_action::NaosoccerPosActionClient" EXECUTABLE naosoccer_pos_action_client)
install(TARGETS
        action_client
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
# End actions

# Test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
