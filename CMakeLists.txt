cmake_minimum_required(VERSION 3.8)
project(imu_utils)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(ACC_LIB_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/acc_lib/allan_acc.cpp
  ${PROJECT_SOURCE_DIR}/src/acc_lib/fitallan_acc.cpp
  )
set(GYR_LIB_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/gyr_lib/allan_gyr.cpp
  ${PROJECT_SOURCE_DIR}/src/gyr_lib/fitallan_gyr.cpp
  )

add_executable(imu_an
  src/imu_an.cpp
  ${GYR_LIB_SOURCE_FILES}
  ${ACC_LIB_SOURCE_FILES}
  )
target_include_directories(imu_an PUBLIC
  src/
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_an PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(imu_an ament_cmake rclcpp sensor_msgs rosbag2_cpp)
target_link_libraries(imu_an ${CERES_LIBRARIES} yaml-cpp)

install(TARGETS imu_an
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
