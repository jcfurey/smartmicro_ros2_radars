cmake_minimum_required(VERSION 3.11)

project(smart_rviz_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(ament_cmake_auto REQUIRED)
find_package(umrr_ros2_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
# Match the Qt major version exported by RViz; mixing Qt 5 and 6 is unsupported.
if(TARGET Qt6::Widgets)
  set(SMART_RVIZ_QT Qt6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(SMART_RVIZ_QT Qt5)
endif()
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(CMAKE_AUTOMOC ON)

set(smart_rviz_plugin_SRCS
  src/smart_recorder.cpp
  src/smart_services.cpp
  src/smart_download.cpp
  src/smart_status.cpp
  src/smart_faults.cpp
  src/umrr96_config.cpp
)

set(smart_rviz_plugin_HDRS
  include/smart_rviz_plugin/smart_recorder.hpp
  include/smart_rviz_plugin/smart_services.hpp
  include/smart_rviz_plugin/smart_download.hpp
  include/smart_rviz_plugin/smart_status.hpp
  include/smart_rviz_plugin/smart_faults.hpp
  include/smart_rviz_plugin/umrr96_config.hpp
)

add_library(smart_rviz_plugin SHARED
  ${smart_rviz_plugin_SRCS}
  ${smart_rviz_plugin_HDRS}
)

target_include_directories(smart_rviz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/smart_rviz_plugin>
  ${OpenCV_INCLUDE_DIRS}
)


target_link_libraries(smart_rviz_plugin
  ${umrr_ros2_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
  ${SMART_RVIZ_QT}::Widgets
)


target_compile_definitions(smart_rviz_plugin PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS smart_rviz_plugin
  EXPORT smart_rviz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(smart_rviz_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(
  pluginlib
  cv_bridge
  OpenCV
  ${SMART_RVIZ_QT}
  rclcpp
  rviz_common
  rviz_ogre_vendor
  sensor_msgs
  umrr_ros2_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  add_executable(panel_loading_smoke test/panel_loading_smoke.cpp)
  target_link_libraries(panel_loading_smoke
    pluginlib::pluginlib
    rclcpp::rclcpp
    rviz_common::rviz_common
    ${SMART_RVIZ_QT}::Widgets
  )
  ament_add_test(panel_loading_smoke
    COMMAND $<TARGET_FILE:panel_loading_smoke>
    TIMEOUT 20
    ENV QT_QPA_PLATFORM=offscreen
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(umrr96_panel_smoke test/umrr96_panel_smoke.cpp)
  target_link_libraries(umrr96_panel_smoke
    ${umrr_ros2_msgs_TARGETS}
    pluginlib::pluginlib
    rclcpp::rclcpp
    rviz_common::rviz_common
    ${SMART_RVIZ_QT}::Widgets
  )
  ament_add_test(umrr96_panel_smoke
    COMMAND $<TARGET_FILE:umrr96_panel_smoke>
    TIMEOUT 30
    ENV QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
