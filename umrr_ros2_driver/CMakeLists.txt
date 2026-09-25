# Copyright 2021 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# The initial version of the code was developed by Apex.AI and
# was thereafter adapted and extended by smartmicro.

cmake_minimum_required(VERSION 3.11)

project(umrr_ros2_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Debug information comes from CMAKE_BUILD_TYPE (Debug/RelWithDebInfo).
  add_compile_options(-Wall -Wextra -Wnarrowing -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)

ament_auto_find_build_dependencies()

# The vendor ships prebuilt Smart Access libraries per platform; only
# lib-linux-x86_64-gcc_9 is known to exist. Override SMARTMICRO_LIB_DIR for
# another vendor-provided directory.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
  set(smartmicro_default_lib_dir "lib-linux-x86_64-gcc_9")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(smartmicro_default_lib_dir "lib-linux-aarch64-gcc_9")
else()
  set(smartmicro_default_lib_dir "lib-linux-${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(SMARTMICRO_LIB_DIR "${smartmicro_default_lib_dir}" CACHE STRING
  "Smart Access library directory relative to smartmicro/")
set(SMARTMICRO_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/smartmicro")
set(SMARTMICRO_SDK_LIB_DIR "${SMARTMICRO_SDK_DIR}/${SMARTMICRO_LIB_DIR}")
if(NOT EXISTS "${SMARTMICRO_SDK_DIR}/include/CommunicationServicesIface.h")
  message(FATAL_ERROR
    "Smart Access SDK is missing. Run ./smart_extract.sh in the repository root.")
endif()
if(NOT EXISTS "${SMARTMICRO_SDK_LIB_DIR}/libsmart_access.so")
  message(FATAL_ERROR
    "No Smart Access libraries for processor '${CMAKE_SYSTEM_PROCESSOR}': "
    "${SMARTMICRO_SDK_LIB_DIR} does not exist. The vendor SDK in this repository "
    "provides prebuilt libraries for x86_64 only (lib-linux-x86_64-gcc_9). Set "
    "-DSMARTMICRO_LIB_DIR=<dir> if smartmicro supplied libraries for this platform.")
endif()

# Check rather than silently modifying an existing vendor installation at build time.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/../tools/patch_smart_access.py"
    --check "${SMARTMICRO_SDK_DIR}"
  RESULT_VARIABLE sdk_patch_result OUTPUT_QUIET ERROR_VARIABLE sdk_patch_error)
if(NOT sdk_patch_result EQUAL 0)
  message(FATAL_ERROR "${sdk_patch_error}\nFrom the repository root run: "
    "python3 tools/patch_smart_access.py umrr_ros2_driver/smartmicro")
endif()

# User interfaces compiled into the data node: <include dir>|<library name>.
set(SMARTMICRO_USER_INTERFACES
  "umrr11_t132_automotive_v1_1_2|umrr11_t132_automotivev1.1.2"
  "umrr96_t153_automotive_v1_2_2|umrr96_t153_automotivev1.2.2"
  "umrr9f_t169_automotive_v1_1_1|umrr9f_t169_automotivev1.1.1"
  "umrr9f_t169_automotive_v2_0_0|umrr9f_t169_automotivev2.0.0"
  "umrr9f_t169_automotive_v2_1_1|umrr9f_t169_automotivev2.1.1"
  "umrr9f_t169_automotive_v2_2_1|umrr9f_t169_automotivev2.2.1"
  "umrr9f_t169_automotive_v2_4_1|umrr9f_t169_automotivev2.4.1"
  "umrr9f_t169_automotive_v3_0_0|umrr9f_t169_automotivev3.0.0"
  "umrr9f_t169_automotive_v3_2_0|umrr9f_t169_automotivev3.2.0"
  "umrr9d_t152_automotive_v1_0_3|umrr9d_t152_automotivev1.0.3"
  "umrr9d_t152_automotive_v1_2_2|umrr9d_t152_automotivev1.2.2"
  "umrr9d_t152_automotive_v1_4_1|umrr9d_t152_automotivev1.4.1"
  "umrr9d_t152_automotive_v1_5_0|umrr9d_t152_automotivev1.5.0"
  "umrr9d_t152_automotive_v1_7_0|umrr9d_t152_automotivev1.7.0"
  "umrra4_automotive_v1_0_1|umrra4_automotivev1.0.1"
  "umrra4_automotive_v1_2_1|umrra4_automotivev1.2.1"
  "umrra4_automotive_v1_4_0|umrra4_automotivev1.4.0"
  "umrra4_automotive_v1_6_0|umrra4_automotivev1.6.0"
  "umrr9f_t169_mse_v1_0_0|umrr9f_t169_msev1.0.0"
  "umrr9f_t169_mse_v1_1_0|umrr9f_t169_msev1.1.0"
  "umrr9f_t169_mse_v1_3_0|umrr9f_t169_msev1.3.0"
  "umrr9f_t169_mse_v2_0_0|umrr9f_t169_msev2.0.0"
  "umrra4_mse_v1_0_0|umrra4_msev1.0.0"
  "umrra4_mse_v2_1_0|umrra4_msev2.1.0"
  "umrra4_mse_v3_0_0|umrra4_msev3.0.0"
  "umrra1_t166_b_automotive_v1_0_0|umrra1_t166_b_automotivev1.0.0"
  "umrra1_t166_b_automotive_v2_0_0|umrra1_t166_b_automotivev2.0.0"
  "umrra1_t166_b_automotive_v2_0_1|umrra1_t166_b_automotivev2.0.1"
  "umrra1_t166_b_automotive_v3_0_0|umrra1_t166_b_automotivev3.0.0")

# Imported targets link the vendor libraries by full path, so build-tree
# binaries (tests) get a build RPATH to them automatically.
function(smartmicro_import_library name)
  add_library(smartmicro_sdk::${name} SHARED IMPORTED)
  set_target_properties(smartmicro_sdk::${name} PROPERTIES
    IMPORTED_LOCATION "${SMARTMICRO_SDK_LIB_DIR}/lib${name}.so"
    INTERFACE_INCLUDE_DIRECTORIES "${SMARTMICRO_SDK_DIR}/include")
endfunction()

set(smartmicro_core_libraries smart_access com_lib osal)
set(smartmicro_installed_libraries ${smartmicro_core_libraries} basev1.0.2_user_interface)
set(smartmicro_ui_include_dirs)
set(smartmicro_ui_targets)
foreach(entry IN LISTS SMARTMICRO_USER_INTERFACES)
  string(REPLACE "|" ";" entry "${entry}")
  list(GET entry 0 include_dir)
  list(GET entry 1 library)
  list(APPEND smartmicro_ui_include_dirs "${SMARTMICRO_SDK_DIR}/include/${include_dir}")
  list(APPEND smartmicro_installed_libraries "${library}_user_interface")
endforeach()
foreach(library IN LISTS smartmicro_installed_libraries)
  smartmicro_import_library(${library})
endforeach()
foreach(entry IN LISTS SMARTMICRO_USER_INTERFACES)
  string(REPLACE "|" ";" entry "${entry}")
  list(GET entry 1 library)
  list(APPEND smartmicro_ui_targets "smartmicro_sdk::${library}_user_interface")
endforeach()
set(smartmicro_core_targets
  smartmicro_sdk::smart_access smartmicro_sdk::com_lib smartmicro_sdk::osal)

# The vendor libraries have generic names (libosal.so, libcom_lib.so...), so
# they are installed privately; installed binaries find them through RPATH.
set(SMARTMICRO_INSTALL_LIB_DIR "lib/${PROJECT_NAME}")
foreach(library IN LISTS smartmicro_installed_libraries)
  install(FILES "${SMARTMICRO_SDK_LIB_DIR}/lib${library}.so"
    DESTINATION "${SMARTMICRO_INSTALL_LIB_DIR}")
endforeach()

# SDK configuration templates and their installed paths. They are generated in
# the build tree and installed; the nodes copy them into a private runtime
# directory and never modify the installed files.
set(LIB_PATH "${CMAKE_INSTALL_PREFIX}/${SMARTMICRO_INSTALL_LIB_DIR}")
set(CONFIG_FOLDER_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config")
set(CONFIG_FILE_PATH "${CONFIG_FOLDER_PATH}/smart_access_config.json")
set(HW_INVENTORY_FILE_PATH "${CONFIG_FOLDER_PATH}/hw_inventory.json")
set(ROUTING_TABLE_FILE_PATH "${CONFIG_FOLDER_PATH}/routing_table.json")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
configure_file(
  "${PROJECT_SOURCE_DIR}/cmake/smart_access_config.json.in"
  "${generated_dir}/config/smart_access_config.json" @ONLY)
configure_file(
  "${PROJECT_SOURCE_DIR}/cmake/config_path.hpp.in"
  "${generated_dir}/include/umrr_ros2_driver/config_path.hpp" @ONLY)
install(FILES "${generated_dir}/config/smart_access_config.json"
  DESTINATION "share/${PROJECT_NAME}/config")
install(FILES "${generated_dir}/include/umrr_ros2_driver/config_path.hpp"
  DESTINATION "include/${PROJECT_NAME}/umrr_ros2_driver")

ament_auto_add_library(smartmicro_radar_node SHARED
  "src/smartmicro_radar_node.cpp"
  "src/update_service.cpp"
)
target_include_directories(smartmicro_radar_node PRIVATE
  "$<BUILD_INTERFACE:${generated_dir}/include>"
  ${smartmicro_ui_include_dirs})
target_link_libraries(smartmicro_radar_node
  nlohmann_json::nlohmann_json
  ${smartmicro_core_targets}
  ${smartmicro_ui_targets})
set_target_properties(smartmicro_radar_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/${PROJECT_NAME}")

rclcpp_components_register_node(smartmicro_radar_node
  PLUGIN "smartmicro::drivers::radar::SmartmicroRadarNode"
  EXECUTABLE smartmicro_radar_node_exe
)

# UMRR-96 control/readback node: a composable component plus a standalone
# executable with the historical name. The generated rclcpp_components main is
# not used for the executable because it aborts on constructor exceptions,
# leaking the private SDK configuration directory and the exit status.
ament_auto_add_library(smartmicro_radar_readback_component SHARED
  src/smartmicro_radar_readback_node.cpp)
target_compile_definitions(smartmicro_radar_readback_component PRIVATE
  SMARTMICRO_SDK_LIBRARY_PATH="${LIB_PATH}")
target_link_libraries(smartmicro_radar_readback_component
  ${smartmicro_core_targets} nlohmann_json::nlohmann_json)
set_target_properties(smartmicro_radar_readback_component PROPERTIES
  INSTALL_RPATH "$ORIGIN/${PROJECT_NAME}")
rclcpp_components_register_nodes(smartmicro_radar_readback_component
  "smartmicro::drivers::radar::ReadbackNode")

ament_auto_add_executable(smartmicro_radar_readback_node
  src/smartmicro_radar_readback_main.cpp)
target_link_libraries(smartmicro_radar_readback_node smartmicro_radar_readback_component)
# Installed in lib/${PROJECT_NAME}: the component is one level up.
set_target_properties(smartmicro_radar_readback_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/..:$ORIGIN")

if(BUILD_TESTING)
    option(SMARTMICRO_NETWORK_TESTS "Enable legacy Docker/network integration tests" OFF)
    if(SMARTMICRO_NETWORK_TESTS)
        find_package(launch_testing_ament_cmake REQUIRED)
        add_launch_test(test/radar_node_test.launch.py TIMEOUT "100")
    endif()

    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_gtest(test_runtime test/test_runtime.cpp)
    target_include_directories(test_runtime PRIVATE include)
    target_link_libraries(test_runtime nlohmann_json::nlohmann_json Threads::Threads)

    ament_add_gtest(test_umrr96_tuning test/test_umrr96_tuning.cpp)
    target_include_directories(test_umrr96_tuning PRIVATE include)
    ament_add_gtest(test_service_parsing test/test_service_parsing.cpp)
    target_include_directories(test_service_parsing PRIVATE include)
    ament_add_gtest(test_udp_socket_health test/test_udp_socket_health.cpp)
    target_include_directories(test_udp_socket_health PRIVATE include)

    ament_add_gtest(test_point_cloud_builder test/test_point_cloud_builder.cpp)
    target_include_directories(test_point_cloud_builder PRIVATE include)
    target_link_libraries(test_point_cloud_builder ${sensor_msgs_TARGETS})
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_point_cloud_builder PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        set_property(TARGET test_point_cloud_builder APPEND_STRING PROPERTY LINK_FLAGS " -fsanitize=address,undefined")
        set_tests_properties(test_point_cloud_builder PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")
    endif()

    ament_add_gtest(test_sdk_float test/test_sdk_float.cpp)
    target_include_directories(test_sdk_float PRIVATE "${SMARTMICRO_SDK_DIR}/include")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_sdk_float PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        set_property(TARGET test_sdk_float APPEND_STRING PROPERTY LINK_FLAGS " -fsanitize=address,undefined")
        set_tests_properties(test_sdk_float PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")
    endif()
    ament_add_pytest_test(test_sdk_patch test/test_sdk_patch.py TIMEOUT 20)

    add_executable(radar_test_sender test/radar_test_sender.cpp)
    target_link_libraries(radar_test_sender ${smartmicro_core_targets})

    set(SMARTMICRO_TEST_DOMAIN_ID 178 CACHE STRING "Isolated ROS domain for offline tests")
    set(offline_environment
        "ROS_DOMAIN_ID=${SMARTMICRO_TEST_DOMAIN_ID}"
        "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs"
        "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
        "CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" multicast=\"false\"/></Interfaces><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>")
    ament_add_pytest_test(test_umrr96_views test/test_umrr96_views.py
        TIMEOUT 60 ENV ${offline_environment})
    ament_add_pytest_test(test_umrr96_measure test/test_umrr96_measure.py
        TIMEOUT 30 ENV ${offline_environment})
    ament_add_pytest_test(test_readback test/readback_smoke.py
        TIMEOUT 60 ENV ${offline_environment})
    ament_add_pytest_test(test_driver_runtime test/test_driver_runtime.py
        TIMEOUT 90 ENV ${offline_environment}
        "SMARTMICRO_TEST_SENDER=$<TARGET_FILE:radar_test_sender>")
    set_tests_properties(test_umrr96_views test_umrr96_measure test_readback test_driver_runtime PROPERTIES RUN_SERIAL TRUE)
       
    # Not applying ros2 linters on external sources
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_copyright
        ament_cmake_lint_cmake
        ament_cmake_uncrustify
        ament_cmake_cpplint
        ament_cmake_flake8
        ament_cmake_cppcheck
        ament_cmake_pep257
    )

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    set(PROJECT_CPP_FILES
      src/smartmicro_radar_node.cpp
      src/update_service.cpp
      include/umrr_ros2_driver/smartmicro_radar_node.hpp
      include/umrr_ros2_driver/update_service.hpp
    )

    set(PROJECT_PY_FILES
      launch/radar.launch.py
      test/radar_node_test.launch.py
    )

    # C++ linting
    ament_cppcheck(${PROJECT_CPP_FILES})

    # Python linting
    ament_pep257(${PROJECT_PY_FILES})

    # Copyright should cover all first-party text sources.
    ament_copyright(
      CMakeLists.txt
      ${PROJECT_CPP_FILES}
      ${PROJECT_PY_FILES}
    )

endif()

install(PROGRAMS scripts/umrr96_views.py DESTINATION lib/${PROJECT_NAME}
    RENAME umrr96_views)
install(PROGRAMS scripts/umrr96_measure.py DESTINATION lib/${PROJECT_NAME}
    RENAME umrr96_measure)
install(FILES scripts/umrr96_fan_image.py scripts/umrr96_filter.py
    DESTINATION lib/${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE config launch param)
