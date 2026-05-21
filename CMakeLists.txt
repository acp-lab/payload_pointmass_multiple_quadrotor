cmake_minimum_required(VERSION 3.8)
project(payload_pointmass_multiple_quadrotor)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(acados_path "$ENV{ACADOS_SOURCE_DIR}")
set(acados_lib "${acados_path}/lib")
set(acados_generated_code "${PROJECT_SOURCE_DIR}/c_generated_code")

if("${acados_path}" STREQUAL "")
  message(FATAL_ERROR "ACADOS_SOURCE_DIR is not set. Please export ACADOS_SOURCE_DIR to your acados root path.")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${acados_generated_code}
  ${acados_path}/interfaces
  ${acados_path}/acados
  ${acados_path}/acados/utils
  ${acados_path}
  ${acados_path}/external
  ${acados_path}/external/blasfeo/include
  ${acados_path}/external/hpipm/include
)

link_directories(${acados_generated_code})

add_library(payload_pointmass_multiple_wrapper SHARED
  src/wrapper_pointmass_multiple.cpp
  src/nmpc_planner_pointmass_multiple.cpp
)

add_library(payload_pointmass_multiple_component SHARED
  src/nodelet_pointmass_multiple.cpp
)

target_include_directories(payload_pointmass_multiple_wrapper
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(payload_pointmass_multiple_wrapper
  Eigen3::Eigen
  libacados_ocp_solver_planner_payload_pointmass_multiple.so
  ${acados_lib}/libacados.so
)
target_link_libraries(payload_pointmass_multiple_component
  payload_pointmass_multiple_wrapper
  Eigen3::Eigen
  libacados_ocp_solver_planner_payload_pointmass_multiple.so
  ${acados_lib}/libacados.so
)

ament_target_dependencies(payload_pointmass_multiple_component
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  std_msgs
)

rclcpp_components_register_node(payload_pointmass_multiple_component
  PLUGIN "payload_pointmass_multiple_quadrotor::NMPCControlPointMassMultipleNodelet"
  EXECUTABLE payload_pointmass_multiple_nodelet_exec)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  payload_pointmass_multiple_wrapper
  payload_pointmass_multiple_component
  payload_pointmass_multiple_nodelet_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/multiple_pointmass_acceleration
  scripts/build_payload_pointmass_multiple
  scripts/reference_publisher_pointmass_multiple
  build_planner_jerk.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(payload_pointmass_multiple_wrapper)
ament_export_libraries(payload_pointmass_multiple_component)
ament_package()
