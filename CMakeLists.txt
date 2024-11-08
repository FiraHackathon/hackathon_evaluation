cmake_minimum_required(VERSION 3.5)
project(hackathon_evaluation)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(tinyxml2_vendor REQUIRED)  # required for find_package(TinyXML2)
find_package(TinyXML2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/evaluation.cpp
  src/crop_field.cpp
  src/xml_world_parser.cpp
  src/crops_viewer.cpp
  src/info_viewer.cpp
)

add_executable(field_coverage_node
  src/field_coverage_node.cpp
  src/field_coverage.cpp
  src/field_grid.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(field_coverage_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  gazebo_msgs
  std_msgs
  visualization_msgs
  rviz_2d_overlay_msgs
)

ament_target_dependencies(field_coverage_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  tinyxml2::tinyxml2
)

target_link_libraries(field_coverage_node
  Eigen3::Eigen
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hackathon::Evaluation"
  EXECUTABLE evaluation_node
  EXECUTOR "SingleThreadedExecutor"
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS field_coverage_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY data config DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
