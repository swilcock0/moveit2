cmake_minimum_required(VERSION 3.5)
project(run_move_group)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# This shouldn't be necessary (required by moveit_simple_controller_manager)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(run_move_group src/run_move_group.cpp)
ament_target_dependencies(run_move_group
  moveit_ros_planning_interface
  Boost
)


add_executable(run_move_group_oop src/run_move_group_oop.cpp)
ament_target_dependencies(run_move_group_oop
  moveit_ros_planning_interface
  Boost
)

install(TARGETS run_move_group
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS run_move_group_oop
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
