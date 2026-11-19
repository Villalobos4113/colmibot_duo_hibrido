cmake_minimum_required(VERSION 3.8)
project(colmibot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)
# Sonda finita de locomoción simulada (C-007): ros2 run colmibot_gazebo motion_probe
install(
  PROGRAMS scripts/motion_probe.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motion_probe
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_static_contract test/test_static_contract.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
    TIMEOUT 180
  )
  ament_add_pytest_test(test_motion_contract test/test_motion_contract.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
    TIMEOUT 180
  )
endif()

ament_package()
