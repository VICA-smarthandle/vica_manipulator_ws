cmake_minimum_required(VERSION 3.8)
project(vica_manipulator)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/go_home.py scripts/load_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
