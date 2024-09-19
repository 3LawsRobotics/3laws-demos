cmake_minimum_required(VERSION 3.5)
project(lll_warehouse_demo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
            launch
            meshes
            models
            rviz
            urdf
            worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
