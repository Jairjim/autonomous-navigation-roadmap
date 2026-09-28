cmake_minimum_required(VERSION 3.8)
project(robotics_cpp_foundations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(robotics_core
            src/vector2d.cpp
            src/pose2d.cpp
            src/angle_utils.cpp
            src/occupancy_grid.cpp
            src/laser_scan.cpp
            src/simulated_lidar.cpp
)

target_include_directories(robotics_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(robotics_core PUBLIC cxx_std_17)

add_executable(robotics_demo
  src/demo.cpp
)

target_link_libraries(robotics_demo
  robotics_core
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    robotics_core
    robotics_demo
  EXPORT export_robotics_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_robotics_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_vector2d
    test/test_vector2d.cpp
  )

  target_link_libraries(test_vector2d
    robotics_core
  )

  ament_add_gtest(test_pose2d
    test/test_pose2d.cpp
  )

  target_link_libraries(test_pose2d
    robotics_core
  )

  ament_add_gtest(test_angle_utils
    test/test_angle_utils.cpp
  )

  target_link_libraries(test_angle_utils
    robotics_core
  )

  ament_add_gtest(test_occupancy_grid
    test/test_occupancy_grid.cpp
  )

  target_link_libraries(test_occupancy_grid
    robotics_core
  )

  ament_add_gtest(test_laser_scan
    test/test_laser_scan.cpp
  )

  target_link_libraries(test_laser_scan
    robotics_core
  )
endif()

ament_package()
