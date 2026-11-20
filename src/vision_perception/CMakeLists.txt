cmake_minimum_required(VERSION 3.8)
project(vision_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  yolo
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  yolo/inference.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_node
)

install(PROGRAMS
  yolo/activity_classifier.py
  yolo/ground_projection.py
  yolo/goal_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
