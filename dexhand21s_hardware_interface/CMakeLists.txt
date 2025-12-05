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
  # dexhand_vendor
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# TODO remove
set(DEXHAND_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../dexhand_sdk_cpp" CACHE PATH "Path to DexHand SDK")

# Check if SDK exists
if(NOT EXISTS ${DEXHAND_SDK_PATH})
  message(FATAL_ERROR "DexHand SDK not found at: ${DEXHAND_SDK_PATH}")
endif()

# Set library and include paths
set(DEXHAND_LIBRARY "${DEXHAND_SDK_PATH}/lib/linux/x86_64/libdexhand.so")
set(SERIAL_LIBRARY "${DEXHAND_SDK_PATH}/lib/linux/x86_64/libserial.so.1")
set(USBCAN_LIBRARY "${DEXHAND_SDK_PATH}/lib/linux/x86_64/libusbcanfd.so.1.0.8")
set(DEXHAND_INCLUDE_DIR "${DEXHAND_SDK_PATH}/include")

add_library(
  dexhand21s_hardware_interface
  SHARED
  src/dexhand21s_hardware_interface.cpp
)

target_include_directories(
  dexhand21s_hardware_interface
  PUBLIC
  include
  ${DEXHAND_INCLUDE_DIR}
)
target_link_libraries(dexhand21s_hardware_interface PUBLIC
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${DEXHAND_LIBRARY}
  ${SERIAL_LIBRARY}
  ${USBCAN_LIBRARY}
  # dexhand_vendor::dexhand_vendor
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
