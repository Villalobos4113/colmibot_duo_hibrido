cmake_minimum_required(VERSION 3.8)
project(colmibot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_static_contract test/test_static_contract.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
    TIMEOUT 180
  )
endif()

ament_package()
