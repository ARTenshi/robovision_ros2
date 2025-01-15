cmake_minimum_required(VERSION 3.8)
project(robovision_images)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${cv_bridge_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# C++ section
add_executable(my_publisher src/my_publisher.cpp)
ament_target_dependencies(my_publisher 
    rclcpp 
    cv_bridge 
    sensor_msgs 
    OpenCV
)

add_executable(my_camera_publisher src/my_camera_publisher.cpp)
ament_target_dependencies(my_camera_publisher 
    rclcpp 
    cv_bridge 
    sensor_msgs 
    OpenCV
)

add_executable(my_subscriber src/my_subscriber.cpp)
ament_target_dependencies(my_subscriber 
    rclcpp 
    cv_bridge 
    sensor_msgs 
    OpenCV
)

# Python section
ament_python_install_package(${PROJECT_NAME})

# Ensure both are included
install(TARGETS
  my_publisher
  my_camera_publisher
  my_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/my_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
