cmake_minimum_required(VERSION 3.14)
project(tag_tracker)

# ── C++ standard ─────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(cv_bridge          REQUIRED)
find_package(image_transport    REQUIRED)

# ── OpenCV ────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

# ── apriltag (system library: sudo apt install libapriltag-dev) ───────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)
link_directories(${APRILTAG_LIBRARY_DIRS})

# ── Library: coordinate_frame ─────────────────────────────────────────────────
add_library(coordinate_frame STATIC
  src/coordinate_frame.cpp
)
target_include_directories(coordinate_frame PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)
target_link_libraries(coordinate_frame
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
)
ament_target_dependencies(coordinate_frame
  rclcpp
  cv_bridge
)

# ── Library: apriltag_detector ────────────────────────────────────────────────
add_library(apriltag_detector STATIC
  src/apriltag_detector.cpp
)
target_include_directories(apriltag_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${APRILTAG_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)
target_link_libraries(apriltag_detector
  ${OpenCV_LIBS}
  ${APRILTAG_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)
ament_target_dependencies(apriltag_detector
  rclcpp
  cv_bridge
  sensor_msgs
)

# ── Executable: tag_tracker_node ─────────────────────────────────────────────
add_executable(tag_tracker_node
  src/tag_tracker_node.cpp
)
target_include_directories(tag_tracker_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
  ${APRILTAG_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)
target_link_libraries(tag_tracker_node
  coordinate_frame
  apriltag_detector
  ${OpenCV_LIBS}
  ${APRILTAG_LIBRARIES}
)
ament_target_dependencies(tag_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS tag_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Linting ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
