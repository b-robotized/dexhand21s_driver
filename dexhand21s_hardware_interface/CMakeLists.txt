cmake_minimum_required(VERSION 3.8)
project(dexhand21s_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Determine architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(DEXHAND_ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(DEXHAND_ARCH "x86_64")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

message(STATUS "Detected architecture: ${DEXHAND_ARCH}")

# Find the DexHand SDK
# Adjust this path to match your workspace structure
set(DEXHAND_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../dexhand_sdk_cpp" CACHE PATH "Path to DexHand SDK")

# Check if SDK exists
if(NOT EXISTS ${DEXHAND_SDK_PATH})
  message(FATAL_ERROR "DexHand SDK not found at: ${DEXHAND_SDK_PATH}")
endif()

# Set library and include paths
set(DEXHAND_LIBRARY "${DEXHAND_SDK_PATH}/lib/linux/${DEXHAND_ARCH}/libdexhand.so")
set(SERIAL_LIBRARY "${DEXHAND_SDK_PATH}/lib/linux/${DEXHAND_ARCH}/libserial.so.1")
set(USBCAN_LIBRARY "${DEXHAND_SDK_PATH}/lib/linux/${DEXHAND_ARCH}/libusbcanfd.so.1.0.8")
set(DEXHAND_INCLUDE_DIR "${DEXHAND_SDK_PATH}/include")

# Verify they exist
if(NOT EXISTS ${DEXHAND_LIBRARY})
  message(FATAL_ERROR "DexHand library not found at: ${DEXHAND_LIBRARY}")
endif()

if(NOT EXISTS ${DEXHAND_INCLUDE_DIR})
  message(FATAL_ERROR "DexHand include directory not found at: ${DEXHAND_INCLUDE_DIR}")
endif()

message(STATUS "Using DexHand library: ${DEXHAND_LIBRARY}")
message(STATUS "Using SERIAL_LIBRARY library: ${SERIAL_LIBRARY}")
message(STATUS "Using USBCAN_LIBRARY library: ${USBCAN_LIBRARY}")
message(STATUS "Using DexHand headers: ${DEXHAND_INCLUDE_DIR}")

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
ament_target_dependencies(
  dexhand21s_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

# Link your driver library
target_link_libraries(dexhand21s_hardware_interface
  ${DEXHAND_LIBRARY}
  ${SERIAL_LIBRARY}
  ${USBCAN_LIBRARY}
)

# prevent pluginlib from using boost
target_compile_definitions(dexhand21s_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

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

if(FALSE)

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
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
