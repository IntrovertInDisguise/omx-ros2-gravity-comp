cmake_minimum_required(VERSION 3.8)
project(omx_dual_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
  launch 
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/verify_dual_setup.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
