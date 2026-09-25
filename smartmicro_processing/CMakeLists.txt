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
  set(SMARTMICRO_PROCESSING_TEST_DOMAIN_ID 178 CACHE STRING "Isolated processing test domain")
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
endif()
ament_package()
