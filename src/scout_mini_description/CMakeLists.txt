cmake_minimum_required(VERSION 3.8)
project(scout_mini_description)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch urdf meshes rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md UPSTREAM.md LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
