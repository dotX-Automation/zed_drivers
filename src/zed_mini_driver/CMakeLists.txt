cmake_minimum_required(VERSION 3.8)
project(zed_mini_driver)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Unusually, we want to disable some warnings because of misshaps in the ZED SDK
  add_compile_options(
    -fopenmp
    -Wall
    -Wextra
    -Wno-deprecated-copy
    -Wno-ignored-qualifiers
    -Wno-unused-parameter)
endif()

# If we're on a Jetson device, some paths and options must be configured to find stubs that
# the Nvidia container runtime will then mount when an executable is loaded
# NOTE: The condition we check is a bit of a hack and might stop working, but for now it work
if(EXISTS "/etc/nv_tegra_release" AND EXISTS "/usr/lib/aarch64-linux-gnu/tegra")
  file(STRINGS "/etc/nv_tegra_release" FILE_CONTENTS LIMIT_COUNT 1)
  string(FIND "${FILE_CONTENTS}" "# R35" MATCH_INDEX)
  if(${MATCH_INDEX} GREATER -1)
    message(STATUS "Jetson platform detected, configuring build to check for CUDA stubs")
    list(APPEND CMAKE_LIBRARY_PATH "/usr/local/cuda/lib64/stubs_")
    list(APPEND CMAKE_CXX_FLAGS "-Wl,--allow-shlib-undefined")
  endif()
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(CUDA REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_interfaces REQUIRED)
find_package(dua_node REQUIRED)
find_package(dua_qos REQUIRED)
find_package(dynamic_systems_control REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(params_manager REQUIRED)
find_package(pose_kit REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(theora_wrappers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ZED 4 REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/zed_mini_driver/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure ZED Mini Driver node source files
file(GLOB ZED_MINI_DRIVER_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/zed_mini_driver/*.cpp")
list(APPEND ZED_MINI_DRIVER_SRCS init_parameters.cpp)

# ZED Mini Driver node
add_library(zed_mini_driver SHARED ${ZED_MINI_DRIVER_SRCS})
target_compile_definitions(zed_mini_driver PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(zed_mini_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDA_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ZED_INCLUDE_DIRS})
target_compile_features(zed_mini_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_directories(zed_mini_driver PUBLIC
  ${CUDA_LIBRARY_DIRS}
  ${ZED_LIBRARY_DIR})
target_link_libraries(zed_mini_driver
  ${CUDA_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${ZED_LIBRARIES}
  Threads::Threads)
ament_target_dependencies(zed_mini_driver
  camera_info_manager
  dua_app_management
  dua_interfaces
  dua_node
  dua_qos
  dynamic_systems_control
  geometry_msgs
  image_transport
  nav_msgs
  params_manager
  pose_kit
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  theora_wrappers
  tf2
  tf2_ros
  visualization_msgs)
rclcpp_components_register_nodes(zed_mini_driver "ZEDMiniDriver::ZEDMiniDriverNode")

# ZED Mini Driver standalone application
add_executable(zed_mini_driver_app
  src/zed_mini_driver_app.cpp)
target_include_directories(zed_mini_driver_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(zed_mini_driver_app PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(zed_mini_driver_app zed_mini_driver)
ament_target_dependencies(zed_mini_driver_app
  dua_app_management
  rclcpp)

# Install ZED Mini Driver component
install(TARGETS zed_mini_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install ZED Mini Driver app
install(TARGETS zed_mini_driver_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

# Make sure that the library path is exported even if the library itself is not
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

ament_package()
