cmake_minimum_required(VERSION 3.5)
project(create3_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

# install(
#  DIRECTORY create3_navigation
#  DESTINATION lib/python3.10/site-packages
# )

# install (PROGRAMS
#    ${PROJECT_NAME}/create3_ir.py
#    DESTINATION lib/${PROJECT_NAME}
# )

# install (PROGRAMS
#   ${PROJECT_NAME}/create3_ir_scan.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# For map_server
# REF: https://answers.ros.org/question/398095/ros2-nav2-map_server-problems-loading-map-with-nav2_map_server/?answer=417598#post-id-417598

# Install the launch file
#    install(DIRECTORY launch/
#    DESTINATION share/${PROJECT_NAME}/launch
# )

# Install the map file
#   install(DIRECTORY maps/
#   DESTINATION share/${PROJECT_NAME}/maps
# )


ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(${PROJECT_NAME}/create3_navigator.py)
# ament_python_install_module(${PROJECT_NAME}/create3_ir_scan.py)
# ament_python_install_module(${PROJECT_NAME}/create3_ir.py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
