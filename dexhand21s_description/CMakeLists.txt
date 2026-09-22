cmake_minimum_required(VERSION 3.5)
project(dexhand21s_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
  meshes urdf rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
