cmake_minimum_required(VERSION 3.8)
project(zed_rgb_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependências do ROS 2 e OpenCV
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_definitions(-DVIDEO_MOD_AVAILABLE)

add_subdirectory(lib/zed-open-capture)

# Inclui os headers da biblioteca
include_directories(
  include
  lib/zed-open-capture/include
)

# Criar o executável
add_executable(zed_rgb_node 
 src/zed_rgb_node.cpp
)

# Linkar com OpenCV e zed-open-capture
target_link_libraries(zed_rgb_node zed_open_capture ${OpenCV_LIBS})

ament_target_dependencies(zed_rgb_node rclcpp sensor_msgs cv_bridge)

# Instalação
install(TARGETS
  zed_rgb_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
