cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
#find_package(rclcpp REQUIRED)
ament_auto_find_build_dependencies()

#add_executable(mpc src/main.cpp)
#ament_target_dependencies(mpc
#  rclcpp
#)
# nodes library



message(STATUS "Debug: Reached this point in CMakeLists.txt")

include_directories(
  ${ACADOS_INSTALL_DIR}/include
  ${ACADOS_INSTALL_DIR}/include/acados
  ${ACADOS_INSTALL_DIR}/include/acados_c  
  ${ACADOS_INSTALL_DIR}/include/blasfeo/include
  ${ACADOS_INSTALL_DIR}/include/hpipm/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mpc_solver  # Add this line

)


link_directories(${ACADOS_INSTALL_DIR}/lib)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/main.cpp
)


# Explicitly add include directories to the target
target_include_directories(${PROJECT_NAME} PUBLIC
  ${ACADOS_INSTALL_DIR}/include
  ${ACADOS_INSTALL_DIR}/include/acados
  ${ACADOS_INSTALL_DIR}/include/acados_c
  ${ACADOS_INSTALL_DIR}/include/blasfeo/include
  ${ACADOS_INSTALL_DIR}/include/hpipm/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mpc_solver  # Add this line
)


# Link ACADOS libraries to the target
target_link_libraries(${PROJECT_NAME}
  acados
  hpipm
  blasfeo
)

# Adicionar o diretório do mpc_solver
add_subdirectory(mpc_solver)


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
