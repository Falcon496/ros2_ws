cmake_minimum_required(VERSION 3.8)
project(gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ignition-cmake2 REQUIRED)
set(IGN_PLUGIN_VER 1)
set(IGN_COMMON_VER 4)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
include_directories(SYSTEM
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)
# Add sources for each plugin to be registered.
add_library(SteerControler src/gz_steer.cpp)
set_property(TARGET SteerControler PROPERTY CXX_STANDARD 17)
target_link_libraries(SteerControler
  ${IGNITION-PLUGIN_LIBRARIES}
  ${IGNITION-GAZEBO_LIBRARIES}
)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_auto_add_executable(
teleop_node src/teleop_node.cpp
)

install(DIRECTORY launch models config params hooks
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

# ament_package()
ament_auto_package()
