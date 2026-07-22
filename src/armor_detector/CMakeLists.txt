cmake_minimum_required(VERSION 3.8)
project(armor_detector)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aim_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn calib3d)

set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime")
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_ROOT}/include")

add_executable(armor_detector_node src/armor_detector_node.cpp)
target_include_directories(armor_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(armor_detector_node
  ament_index_cpp aim_interfaces cv_bridge rclcpp sensor_msgs)
target_link_libraries(armor_detector_node ${OpenCV_LIBS} onnxruntime)
set_target_properties(armor_detector_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS armor_detector_node DESTINATION lib/${PROJECT_NAME})
install(
  FILES third_party/onnxruntime/lib/libonnxruntime.so
  DESTINATION lib
  RENAME libonnxruntime.so.1.18.1)
install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
