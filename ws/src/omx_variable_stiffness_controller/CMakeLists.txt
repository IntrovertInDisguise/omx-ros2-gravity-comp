cmake_minimum_required(VERSION 3.8)
project(omx_variable_stiffness_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
# these are used by the launch tests
# the two message packages are only required by the Python tests
# (they provide request/response types and JointState messages), so defer
# finding/linking them until BUILD_TESTING is enabled below.

add_library(${PROJECT_NAME} SHARED
  src/omx_variable_stiffness_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  pluginlib
  hardware_interface
  kdl_parser
  urdf
  geometry_msgs
  std_msgs
  tf2_kdl
  tf2_eigen
)

target_link_libraries(${PROJECT_NAME}
  orocos-kdl
  Eigen3::Eigen
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(controller_interface plugin.xml)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/load_stiffness.py
  scripts/logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# tests
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # the python tests rely on xacro
  find_package(xacro REQUIRED)
  # message types only needed by the tests
  find_package(sensor_msgs REQUIRED)
  find_package(controller_manager_msgs REQUIRED)

  ament_add_pytest_test(test_urdf_sim_plugin test/test_urdf_sim_plugin.py
    TIMEOUT 30)
  ament_add_pytest_test(test_controller_manager test/test_controller_manager.py
    TIMEOUT 60)
  ament_add_pytest_test(test_simulation_launch test/test_simulation_launch.py
    TIMEOUT 120)
endif()

ament_package()
