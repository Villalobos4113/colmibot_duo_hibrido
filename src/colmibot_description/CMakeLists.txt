cmake_minimum_required(VERSION 3.8)
project(colmibot_description)

find_package(ament_cmake REQUIRED)

# meshes/ se instala aunque esté vacío (clon sin mallas); ver SOURCE.md.
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitignore" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # El xacro usa $(find colmibot_description): se añade el prefijo propio al índice ament.
  ament_add_pytest_test(test_model test/test_model.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
    TIMEOUT 120
  )
endif()

ament_package()
