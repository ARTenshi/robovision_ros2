cmake_minimum_required(VERSION 3.8)
project(robovision_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robovision_interfaces REQUIRED)

include_directories(${cv_bridge_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# C++ section
add_executable(robovision_service src/robovision_service.cpp)
ament_target_dependencies(robovision_service 
    rclcpp 
    cv_bridge 
    sensor_msgs 
    geometry_msgs
    robovision_interfaces
    OpenCV
    tf2
    tf2_geometry_msgs
)

add_executable(robovision_client src/robovision_client.cpp)
ament_target_dependencies(robovision_client 
    rclcpp 
    cv_bridge 
    sensor_msgs 
    geometry_msgs
    robovision_interfaces
    OpenCV
    tf2
    tf2_geometry_msgs
)

# Python section
ament_python_install_package(${PROJECT_NAME})

# Ensure both are included
install(TARGETS
robovision_service
robovision_client
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/robovision_service.py
  scripts/robovision_client.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
