cmake_minimum_required(VERSION 3.8)
project(control_utils)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# Cria uma biblioteca com o teu código
add_library(${PROJECT_NAME}
  src/coordinate_transform.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/nanoflann/include>
    $<INSTALL_INTERFACE:include>
)

# Exporta a biblioteca e includes para outros pacotes
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY lib/nanoflann/include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
