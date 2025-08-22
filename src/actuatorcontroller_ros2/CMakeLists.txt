cmake_minimum_required(VERSION 3.8)
project(actuatorcontroller_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add C++17 support
add_compile_options(-std=c++17)
add_compile_options(-fPIC)
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)
add_compile_options(-DIMU_ENABLE)
add_compile_options(-O3)

# Default to Release build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

MESSAGE("CMAKE_SYSTEM_PROCESSOR:   ${CMAKE_SYSTEM_PROCESSOR}")

if( "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
 set(BUILD_PLATFORM
         X86)
else()
 set(BUILD_PLATFORM
         TX2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Messages
set(msg_files
  "msg/ActuatorAttribute.msg"
  "msg/ActuatorCommand.msg"
  "msg/ActuatorModes.msg"
  "msg/ActuatorArray.msg"
)

# Services
set(srv_files
  "srv/AttributeLookup.srv"
  "srv/AttributeQuery.srv"
  "srv/GeneralQuery.srv"
  "srv/AttributeDictionary.srv"
  "srv/DebugQuery.srv"
  "srv/TriviaQuery.srv"
  "srv/IDModify.srv"
  "srv/ParametersSave.srv"
  "srv/ZeroReset.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
  include/${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/include
)

# Copy the ActuatorController SDK library to the build directory
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/lib/${BUILD_PLATFORM}/libActuatorController.so
  ${CMAKE_CURRENT_BINARY_DIR}/libActuatorController.so
  COPYONLY
)

# Library
add_library(actuatorcontroller_lib
  src/${PROJECT_NAME}/actuatorcontroller_ros2.cpp
  src/${PROJECT_NAME}/actuatorcontroller_ros2param.cpp
  src/${PROJECT_NAME}/actuatorcontroller_ros2service.cpp
  src/${PROJECT_NAME}/trajectory_subscriber.cpp
)

# Add dependencies
ament_target_dependencies(actuatorcontroller_lib
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  trajectory_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(actuatorcontroller_lib
  ${cpp_typesupport_target}
  ${CMAKE_CURRENT_BINARY_DIR}/libActuatorController.so
)

# Executable
add_executable(innfos_actuator src/innfos_actuator.cpp)
target_link_libraries(innfos_actuator 
  actuatorcontroller_lib
  ${CMAKE_CURRENT_BINARY_DIR}/libActuatorController.so
)
ament_target_dependencies(innfos_actuator
  rclcpp
  trajectory_msgs
)

# Install
install(TARGETS
  actuatorcontroller_lib
  innfos_actuator
  DESTINATION lib/${PROJECT_NAME}
)

# Install the ActuatorController SDK library
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/lib/${BUILD_PLATFORM}/libActuatorController.so
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  ActuatorController_SDK/
  DESTINATION share/${PROJECT_NAME}/ActuatorController_SDK
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export
ament_export_include_directories(include)
ament_export_libraries(actuatorcontroller_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  trajectory_msgs
)

# Add the library directory to the runtime path
set_target_properties(actuatorcontroller_lib innfos_actuator PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()