cmake_minimum_required(VERSION 3.16)

project(smart_rviz_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(umrr_ros2_msgs REQUIRED)
find_package(rviz_common REQUIRED)
# Match the Qt major version exported by RViz; mixing Qt 5 and 6 is unsupported.
if(TARGET Qt6::Widgets)
  set(SMART_RVIZ_QT Qt6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(SMART_RVIZ_QT Qt5)
endif()
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)

set(smart_rviz_plugin_SRCS
  src/smart_recorder.cpp
  src/smart_services.cpp
  src/smart_download.cpp
  src/smart_status.cpp
  src/smart_faults.cpp
  src/umrr96_config.cpp
  src/umrr96_advanced.cpp
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

# Headers are installed to include/${PROJECT_NAME}/smart_rviz_plugin/*.hpp, so both
# interfaces expose the same "smart_rviz_plugin/<header>.hpp" include path.
target_include_directories(smart_rviz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(smart_rviz_plugin
  ${umrr_ros2_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  ${SMART_RVIZ_QT}::Widgets
)

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
  DESTINATION include/${PROJECT_NAME}
)

# Instruction tables for the Command Configurator panel. They ship with the Smart Access
# SDK (smart_extract.sh unpacks it into umrr_ros2_driver/smartmicro); install the tables
# so the panel does not depend on RViz's working directory.
set(SMART_USER_INTERFACES_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../umrr_ros2_driver/smartmicro/user_interfaces"
  CACHE PATH "Smart Access user_interfaces directory with the sensor instruction tables")
if(EXISTS "${SMART_USER_INTERFACES_DIR}")
  install(
    DIRECTORY "${SMART_USER_INTERFACES_DIR}/"
    DESTINATION share/${PROJECT_NAME}/user_interfaces
    FILES_MATCHING
    PATTERN "*.param"
    PATTERN "*.command"
    PATTERN "*.status"
  )
else()
  message(WARNING "Smart Access user_interfaces not found at ${SMART_USER_INTERFACES_DIR}; "
    "the Command Configurator panel will show no instruction tables. Run smart_extract.sh.")
endif()

ament_export_targets(smart_rviz_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  pluginlib
  ${SMART_RVIZ_QT}
  rcl_interfaces
  rclcpp
  rviz_common
  sensor_msgs
  std_msgs
  umrr_ros2_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  # The C++ panels predate ament's C++ style checks; lint Python, XML and CMake only.
  ament_flake8(custom_can_sender.py)
  ament_pep257(custom_can_sender.py)
  ament_xmllint()
  ament_lint_cmake()

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
    ${rcl_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    pluginlib::pluginlib
    rclcpp::rclcpp
    rviz_common::rviz_common
    ${SMART_RVIZ_QT}::Widgets
  )
  ament_add_test(umrr96_panel_smoke
    COMMAND $<TARGET_FILE:umrr96_panel_smoke>
    TIMEOUT 60
    ENV QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(generic_panels_smoke test/generic_panels_smoke.cpp)
  target_link_libraries(generic_panels_smoke
    ${umrr_ros2_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    pluginlib::pluginlib
    rclcpp::rclcpp
    rviz_common::rviz_common
    ${SMART_RVIZ_QT}::Widgets
  )
  ament_add_test(generic_panels_smoke
    COMMAND $<TARGET_FILE:generic_panels_smoke>
    TIMEOUT 90
    ENV QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=176 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      SMART_USER_INTERFACES_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/user_interfaces
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
