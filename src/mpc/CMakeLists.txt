cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

message(STATUS "ACADOS_INSTALL_DIR = $ENV{ACADOS_INSTALL_DIR}")

include_directories(
  $ENV{ACADOS_INSTALL_DIR}/include
  $ENV{ACADOS_INSTALL_DIR}/include/acados
  $ENV{ACADOS_INSTALL_DIR}/include/acados_c  
  $ENV{ACADOS_INSTALL_DIR}/include/blasfeo/include
  $ENV{ACADOS_INSTALL_DIR}/include/hpipm/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mpc_solver  # Add this line
)


link_directories($ENV{ACADOS_INSTALL_DIR}/lib)

set(BUILD_ACADOS_SOLVER_LIB ON CACHE INTERNAL "")
add_subdirectory(mpc_solver ${CMAKE_CURRENT_BINARY_DIR}/mpc_solver EXCLUDE_FROM_ALL)
#add_library(acados_solver_mpc_model SHARED ...)

ament_auto_add_executable(mpc
  src/main.cpp
)

#add_subdirectory(mpc_solver)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/mpc_solver
)


# Explicitly add include directories to the target
#target_include_directories(${PROJECT_NAME} PUBLIC
#  ${ACADOS_INSTALL_DIR}/include
#  ${ACADOS_INSTALL_DIR}/include/acados
#  ${ACADOS_INSTALL_DIR}/include/acados_c
#  ${ACADOS_INSTALL_DIR}/include/blasfeo/include
#  ${ACADOS_INSTALL_DIR}/include/hpipm/include
#  ${CMAKE_CURRENT_SOURCE_DIR}/mpc_solver  # Add this line
#)

#add_library(acados_solver_mpc_model SHARED ...)


# Link ACADOS libraries to the target
target_link_libraries(${PROJECT_NAME}
  acados
  hpipm
  blasfeo
  acados_solver_mpc_model
)

# Adicionar o diretório do mpc_solver
#set(BUILD_ACADOS_SOLVER_LIB ON CACHE INTERNAL "")
#add_subdirectory(mpc_solver)


# Install the executable
install(TARGETS ${PROJECT_NAME}
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

# Copy the shared library manually into the install/lib/ directory
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/lib
  COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_BINARY_DIR}/mpc_solver/libacados_solver_mpc_model.so
    ${CMAKE_INSTALL_PREFIX}/lib/libacados_solver_mpc_model.so
)

# Configurar testes (opcional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
