cmake_minimum_required(VERSION 3.5)

project(qrb_ros_vision_service)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_executable(vision_service
  src/main.cpp
  src/action_handler.cpp
  src/vision_service.cpp
)

target_link_libraries(vision_service
  qrb_vision_manager
  yaml-cpp
  zbar
)

ament_auto_package()