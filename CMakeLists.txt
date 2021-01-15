cmake_minimum_required(VERSION 3.5)
project(minirys_global_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
set(CMAKE_PREFIX_PATH "/home/dangield/opencv3/install")
find_package(OpenCV REQUIRED)
set(CMAKE_PREFIX_PATH "/home/dangield/aruco/install")
find_package(aruco REQUIRED)

# including headers
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${aruco_INCLUDE_DIRS} )
include_directories( /usr/include/flycapture )
install(
        DIRECTORY include/
        DESTINATION include
)

# setting ament dependencies
set(AMENT_DEPENDENCIES
	rclcpp
)

add_executable(camera_test src/camera_test.cpp)
target_link_libraries(camera_test flycapture)
ament_target_dependencies(camera_test ${AMENT_DEPENDENCIES})

target_include_directories(camera_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(detection_test src/detection_test.cpp)
target_link_libraries(detection_test flycapture ${OpenCV_LIBS} aruco)
ament_target_dependencies(detection_test ${AMENT_DEPENDENCIES})

target_include_directories(detection_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(detection_consistency_test src/detection_consistency_test.cpp)
target_link_libraries(detection_consistency_test flycapture ${OpenCV_LIBS} aruco)
ament_target_dependencies(detection_consistency_test ${AMENT_DEPENDENCIES})

target_include_directories(detection_consistency_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


install(TARGETS
  camera_test
  detection_test
  detection_consistency_test
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
