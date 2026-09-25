cmake_minimum_required(VERSION 3.11)
project(smartmicro_processing)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/umrr96_processing scripts/umrr96_processing_audit
  scripts/umrr96_accumulation scripts/umrr96_accumulation_audit
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md ACCUMULATION.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_doppler test/test_doppler.py TIMEOUT 30)
  ament_add_pytest_test(test_cloud test/test_cloud.py TIMEOUT 30)
  ament_add_pytest_test(test_accumulation test/test_accumulation.py TIMEOUT 30)
  ament_add_pytest_test(test_ghosts test/test_ghosts.py TIMEOUT 30)
  ament_add_pytest_test(test_tracker test/test_tracker.py TIMEOUT 30)
  ament_add_pytest_test(test_obstacles test/test_obstacles.py TIMEOUT 30)
  ament_add_pytest_test(test_vectorized_equivalence test/test_vectorized_equivalence.py
    TIMEOUT 60)
  set(SMARTMICRO_PROCESSING_TEST_DOMAIN_ID 178 CACHE STRING "Isolated processing test domain")
  ament_add_pytest_test(test_node_logic test/test_node_logic.py
    TIMEOUT 60 ENV
    "ROS_DOMAIN_ID=${SMARTMICRO_PROCESSING_TEST_DOMAIN_ID}"
    "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
    "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs"
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
  ament_add_pytest_test(test_processing_runtime test/test_processing_runtime.py
    TIMEOUT 60 ENV
    "ROS_DOMAIN_ID=${SMARTMICRO_PROCESSING_TEST_DOMAIN_ID}"
    "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
    "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs"
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    "CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" multicast=\"false\"/></Interfaces><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>")
  set(SMARTMICRO_ACCUMULATION_TEST_DOMAIN_ID 177 CACHE STRING "Isolated accumulation test domain")
  ament_add_pytest_test(test_accumulation_runtime test/test_accumulation_runtime.py
    TIMEOUT 60 ENV
    "ROS_DOMAIN_ID=${SMARTMICRO_ACCUMULATION_TEST_DOMAIN_ID}"
    "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
    "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs"
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    "CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" multicast=\"false\"/></Interfaces><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>")

  # Lint first-party sources; files carry SPDX identifiers but no copyright
  # notices, so ament_copyright is not applied.
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_flake8()
  ament_flake8(TESTNAME flake8_scripts
    scripts/umrr96_processing scripts/umrr96_processing_audit
    scripts/umrr96_accumulation scripts/umrr96_accumulation_audit)
  ament_pep257()
  ament_xmllint()
  ament_lint_cmake()
endif()
ament_package()
