cmake_minimum_required(VERSION 3.8)
project(romr_description)

find_package(ament_cmake REQUIRED)

# Install assets into the package share
install(DIRECTORY urdf meshes gazebo launch worlds resource
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git" EXCLUDE
)

# Register in ament index (so ros2 can discover the package)
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

# Also ship package.xml in the share dir
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
