cmake_minimum_required(VERSION 3.8)
project(wave_rover_base_cpp)

# colcon geeft standaard geen buildtype mee: zonder optimalisatie is de winst kleiner.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(esp32_bridge src/esp32_bridge.cpp)
target_compile_features(esp32_bridge PUBLIC cxx_std_17)
target_link_libraries(esp32_bridge
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

install(TARGETS esp32_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
