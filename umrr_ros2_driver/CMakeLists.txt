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
    add_compile_options(-Wall -Wextra -Wnarrowing -Wpedantic -Wno-unused-parameter -g)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)

ament_auto_find_build_dependencies()

set(SMARTMICRO_LIB_DIR "lib-linux-x86_64-gcc_9" CACHE STRING
    "Smart Access library directory relative to smartmicro/")
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libsmart_access.so")
    message(FATAL_ERROR "Smart Access SDK is missing. Run ./smart_extract.sh in the repository root.")
endif()

# Check rather than silently modifying an existing vendor installation at build time.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/../tools/patch_smart_access.py"
        --check "${PROJECT_SOURCE_DIR}/smartmicro"
    RESULT_VARIABLE sdk_patch_result OUTPUT_QUIET ERROR_VARIABLE sdk_patch_error)
if(NOT sdk_patch_result EQUAL 0)
    message(FATAL_ERROR "${sdk_patch_error}\nFrom the repository root run: python3 tools/patch_smart_access.py umrr_ros2_driver/smartmicro")
endif()

# specify the path to search for the libraries
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR})

install(FILES
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libsmart_access.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libosal.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libcom_lib.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libbasev1.0.2_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr11_t132_automotivev1.1.2_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr96_t153_automotivev1.2.2_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev1.1.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev2.0.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev2.1.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev2.2.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev2.4.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev3.0.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev3.2.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9d_t152_automotivev1.0.3_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9d_t152_automotivev1.2.2_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9d_t152_automotivev1.4.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9d_t152_automotivev1.5.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9d_t152_automotivev1.7.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra4_automotivev1.0.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra4_automotivev1.2.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra4_automotivev1.4.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra4_automotivev1.6.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_msev1.0.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_msev1.1.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_msev1.3.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_msev2.0.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra4_msev1.0.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra4_msev2.1.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra4_msev3.0.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra1_t166_b_automotivev1.0.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra1_t166_b_automotivev2.0.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra1_t166_b_automotivev2.0.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra1_t166_b_automotivev3.0.0_user_interface.so"
DESTINATION lib)

set(LIB_PATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CONFIG_FOLDER_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config")
set(CONFIG_FILE_PATH "${CONFIG_FOLDER_PATH}/smart_access_config.json")
set(HW_INVENTORY_FILE_PATH "${CONFIG_FOLDER_PATH}/hw_inventory.json")
set(ROUTING_TABLE_FILE_PATH "${CONFIG_FOLDER_PATH}/routing_table.json")
configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/smart_access_config.json.in" "${CONFIG_FILE_PATH}" @ONLY)
configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/config_path.hpp.in"
    "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/config_path.hpp" @ONLY)

ament_auto_add_library(smartmicro_radar_node SHARED 
    "src/smartmicro_radar_node.cpp"
    "src/update_service.cpp"
    )

target_include_directories(smartmicro_radar_node PUBLIC ${CMAKE_INSTALL_PREFIX}/include/
smartmicro/include/
smartmicro/include/umrr11_t132_automotive_v1_1_2
smartmicro/include/umrr96_t153_automotive_v1_2_2
smartmicro/include/umrr9f_t169_automotive_v1_1_1
smartmicro/include/umrr9f_t169_automotive_v2_0_0
smartmicro/include/umrr9f_t169_automotive_v2_1_1
smartmicro/include/umrr9f_t169_automotive_v2_2_1
smartmicro/include/umrr9f_t169_automotive_v2_4_1
smartmicro/include/umrr9f_t169_automotive_v3_0_0
smartmicro/include/umrr9f_t169_automotive_v3_2_0
smartmicro/include/umrr9d_t152_automotive_v1_0_3
smartmicro/include/umrr9d_t152_automotive_v1_2_2
smartmicro/include/umrr9d_t152_automotive_v1_4_1
smartmicro/include/umrr9d_t152_automotive_v1_5_0
smartmicro/include/umrr9d_t152_automotive_v1_7_0
smartmicro/include/umrra4_automotive_v1_0_1
smartmicro/include/umrra4_automotive_v1_2_1
smartmicro/include/umrra4_automotive_v1_4_0
smartmicro/include/umrra4_automotive_v1_6_0
smartmicro/include/umrr9f_t169_mse_v1_0_0
smartmicro/include/umrr9f_t169_mse_v1_1_0
smartmicro/include/umrr9f_t169_mse_v1_3_0
smartmicro/include/umrr9f_t169_mse_v2_0_0
smartmicro/include/umrra4_mse_v1_0_0
smartmicro/include/umrra4_mse_v2_1_0
smartmicro/include/umrra4_mse_v3_0_0
smartmicro/include/umrra1_t166_b_automotive_v1_0_0
smartmicro/include/umrra1_t166_b_automotive_v2_0_0
smartmicro/include/umrra1_t166_b_automotive_v2_0_1
smartmicro/include/umrra1_t166_b_automotive_v3_0_0)

# link smart_access_lib-linux-x86_64-gcc_9 to the node
target_link_libraries(smartmicro_radar_node
    smart_access
    nlohmann_json::nlohmann_json
    com_lib
    osal
    umrr11_t132_automotivev1.1.2_user_interface
    umrr96_t153_automotivev1.2.2_user_interface
    umrr9f_t169_automotivev1.1.1_user_interface
    umrr9f_t169_automotivev2.0.0_user_interface
    umrr9f_t169_automotivev2.1.1_user_interface
    umrr9f_t169_automotivev2.2.1_user_interface
    umrr9f_t169_automotivev2.4.1_user_interface
    umrr9f_t169_automotivev3.0.0_user_interface
    umrr9f_t169_automotivev3.2.0_user_interface
    umrr9d_t152_automotivev1.0.3_user_interface
    umrr9d_t152_automotivev1.2.2_user_interface
    umrr9d_t152_automotivev1.4.1_user_interface
    umrr9d_t152_automotivev1.5.0_user_interface
    umrr9d_t152_automotivev1.7.0_user_interface
    umrra4_automotivev1.0.1_user_interface
    umrra4_automotivev1.2.1_user_interface
    umrra4_automotivev1.4.0_user_interface
    umrra4_automotivev1.6.0_user_interface
    umrr9f_t169_msev1.0.0_user_interface
    umrr9f_t169_msev1.1.0_user_interface
    umrr9f_t169_msev1.3.0_user_interface
    umrr9f_t169_msev2.0.0_user_interface
    umrra4_msev1.0.0_user_interface
    umrra4_msev2.1.0_user_interface
    umrra4_msev3.0.0_user_interface
    umrra1_t166_b_automotivev1.0.0_user_interface
    umrra1_t166_b_automotivev2.0.0_user_interface
    umrra1_t166_b_automotivev2.0.1_user_interface
    umrra1_t166_b_automotivev3.0.0_user_interface)

rclcpp_components_register_node(smartmicro_radar_node
    PLUGIN "smartmicro::drivers::radar::SmartmicroRadarNode"
    EXECUTABLE smartmicro_radar_node_exe
)

ament_auto_add_executable(smartmicro_radar_readback_node
    src/smartmicro_radar_readback_node.cpp)
target_include_directories(smartmicro_radar_readback_node PRIVATE smartmicro/include)
target_compile_definitions(smartmicro_radar_readback_node PRIVATE
    SMARTMICRO_SDK_LIBRARY_PATH="${CMAKE_INSTALL_PREFIX}/lib")
target_link_libraries(smartmicro_radar_readback_node
    smart_access com_lib osal nlohmann_json::nlohmann_json)

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
    target_include_directories(test_sdk_float PRIVATE smartmicro/include)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_sdk_float PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        set_property(TARGET test_sdk_float APPEND_STRING PROPERTY LINK_FLAGS " -fsanitize=address,undefined")
        set_tests_properties(test_sdk_float PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")
    endif()
    ament_add_pytest_test(test_sdk_patch test/test_sdk_patch.py TIMEOUT 20)

    add_executable(radar_test_sender test/radar_test_sender.cpp)
    target_include_directories(radar_test_sender PRIVATE smartmicro/include)
    target_link_libraries(radar_test_sender smart_access com_lib osal)

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
    ament_add_pytest_test(test_umrr96_python_lint test/test_umrr96_python_lint.py TIMEOUT 120)
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

# Python modules (views node, detection filter, fan image) are an importable
# package; the installed executable names are unchanged.
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(umrr_ros2_driver_py)
install(PROGRAMS scripts/umrr96_views.py DESTINATION lib/${PROJECT_NAME}
    RENAME umrr96_views)
install(PROGRAMS scripts/umrr96_measure.py DESTINATION lib/${PROJECT_NAME}
    RENAME umrr96_measure)

ament_auto_package(INSTALL_TO_SHARE config launch param)
