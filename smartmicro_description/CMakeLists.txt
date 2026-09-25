cmake_minimum_required(VERSION 3.11)
project(smartmicro_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_flake8()
  ament_pep257()
  ament_xmllint()
  ament_lint_cmake()
  ament_add_pytest_test(test_umrr96_description test/test_umrr96_description.py TIMEOUT 30)
  set(SMARTMICRO_DESCRIPTION_TEST_DOMAIN_ID 179 CACHE STRING "Isolated description test domain")
  ament_add_pytest_test(test_umrr96_description_runtime test/test_umrr96_description_runtime.py
    TIMEOUT 45 ENV
    "ROS_DOMAIN_ID=${SMARTMICRO_DESCRIPTION_TEST_DOMAIN_ID}"
    "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
    "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs"
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    "CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" multicast=\"false\"/></Interfaces><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>")
endif()

ament_package()
