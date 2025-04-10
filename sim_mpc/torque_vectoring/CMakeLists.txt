cmake_minimum_required(VERSION 3.0.2)
project(torque_vectoring)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  common_msgs
)


catkin_package(
   INCLUDE_DIRS include
   INCLUDE_DIRS solverTV/include
   INCLUDE_DIRS solverTV
   LIBRARIES torque_vectoring
)


include_directories(
  include
  solverTV/include
  solverTV
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_torque_vectoring src/tqvec_main.cpp src/tqvec_handle.cpp src/tqvec_pipeline.cpp solverTV/solverTV_adtool2forces.c solverTV/solverTV_casadi.c
)
set_target_properties(
  ${PROJECT_NAME}_torque_vectoring PROPERTIES OUTPUT_NAME torque_vectoring PREFIX ""
)
add_dependencies(
  ${PROJECT_NAME}_torque_vectoring ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_torque_vectoring ${PROJECT_NAME}_pid ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_pid src/tqvec_pid.cpp)
add_dependencies(${PROJECT_NAME}_pid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_pid ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_libsolverTV SHARED IMPORTED)
set_target_properties(${PROJECT_NAME}_libsolverTV PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/solverTV/lib/libsolverTV.so)
set_target_properties(${PROJECT_NAME}_libsolverTV PROPERTIES LINKER_LANGUAGE C)
target_link_libraries(${PROJECT_NAME}_torque_vectoring ${PROJECT_NAME}_libsolverTV)
