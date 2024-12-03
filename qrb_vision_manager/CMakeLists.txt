cmake_minimum_required(VERSION 3.8)
project(qrb_vision_manager VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -std=c++17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

set(LIBRARY_SOURCES
  src/camera_manager.cpp
  src/mission_manager.cpp
  src/parameter_manager.cpp
  src/request_manager.cpp
  src/executor/camera.cpp
  src/executor/executor_base.cpp
  src/executor/qr_code_detect.cpp
  src/executor/video.cpp
  src/executor/object_detect.cpp
  src/executor/image_segment.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${LIBRARY_SOURCES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(FILES config/vision_manager_node.conf
  DESTINATION /etc/${PROJECT_NAME}
)

ament_auto_package()
