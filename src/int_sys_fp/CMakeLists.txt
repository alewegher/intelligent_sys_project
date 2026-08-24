cmake_minimum_required(VERSION 3.8)
project(int_sys_fp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Find yaml-cpp
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# Generate custom messages PRIMA di definire gli eseguibili
rosidl_generate_interfaces(${PROJECT_NAME}
  "custom_messages/RobotDist.msg"
  "custom_messages/AnchorDist.msg"
  "custom_messages/FsmState.msg"
  "custom_messages/PoseEstimateDebug.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Add executable
add_executable(uwb_emulator src/UWB_utils_emulator.cpp)
add_executable(regulator_node src/Regulator_node.cpp)
add_executable(distance_kf_node src/KF.cpp)
add_executable(pose_ukf_node src/UKF.cpp)
add_executable(pose_ekf_node src/PoseEKF_node.cpp)

# --- CONFIGURAZIONE DEGLI HEADER E DELLE DIPENDENZE ---
# Imposta la cartella 'include' per i nodi sorgente (CORREZIONE per percorsi relativi)
# Utilizza CMAKE_CURRENT_SOURCE_DIR per fornire un percorso assoluto alla cartella 'include'
target_include_directories(uwb_emulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(regulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(distance_kf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(pose_ukf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(pose_ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Link dependencies (ROS 2 dependencies)
ament_target_dependencies(uwb_emulator
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  ament_index_cpp
)

ament_target_dependencies(regulator_node
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
)

ament_target_dependencies(distance_kf_node
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
)

ament_target_dependencies(pose_ukf_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  ament_index_cpp
)

ament_target_dependencies(pose_ekf_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  ament_index_cpp
)

# Link with generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(uwb_emulator ${cpp_typesupport_target})
target_link_libraries(regulator_node ${cpp_typesupport_target})
target_link_libraries(distance_kf_node ${cpp_typesupport_target})
target_link_libraries(pose_ukf_node ${cpp_typesupport_target})
target_link_libraries(pose_ekf_node ${cpp_typesupport_target})

# Link with external libraries (YAML and Eigen)
target_link_libraries(uwb_emulator
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(regulator_node
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(distance_kf_node
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(pose_ukf_node
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(pose_ekf_node
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
)
# -----------------------------------------------------


# Install executable
install(TARGETS
  uwb_emulator
  regulator_node
  distance_kf_node
  pose_ukf_node
  pose_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/KF.py
  src/trajectory_planner.py
  src/run_timer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files to share/${PROJECT_NAME}/config/, matching the source layout
# (code reads them via get_package_share_directory(...) + "/config/<name>.yaml").
# Deliberately install(FILES ...) and not install(DIRECTORY ...): colcon's
# --symlink-install symlinks individual files listed here, but only copies files
# reached through install(DIRECTORY ...) - using DIRECTORY would silently bring back
# the "edit needs a rebuild" problem for every YAML in config/.
install(FILES
  config/sensor_params.yaml
  config/sensor_params_uniform.yaml
  config/UKF_params.yaml
  config/controller.yaml
  config/pose_filter_params.yaml
  DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies for message generation
ament_export_dependencies(rosidl_default_runtime)

ament_package()