cmake_minimum_required(VERSION 3.8)
project(dexhand21s_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(ENABLE_TESTING FALSE)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  dexhand_vendor
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  dexhand21s_hardware_interface
  SHARED
  src/dexhand21s_hardware_interface.cpp
)

target_include_directories(
  dexhand21s_hardware_interface
  PUBLIC
  include
  ${dexhand_vendor_INCLUDE_DIRS}
)
target_link_libraries(dexhand21s_hardware_interface PUBLIC
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${dexhand_vendor_LIBRARIES}
  serial
)

pluginlib_export_plugin_description_file(
  hardware_interface dexhand21s_hardware_interface.xml)

install(
  TARGETS
  dexhand21s_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(ENABLE_TESTING)

  ament_add_gmock(test_dexhand21s_hardware_interface test/test_dexhand21s_hardware_interface.cpp)
  target_include_directories(test_dexhand21s_hardware_interface PRIVATE include)
  ament_target_dependencies(
    test_dexhand21s_hardware_interface
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  dexhand21s_hardware_interface
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
