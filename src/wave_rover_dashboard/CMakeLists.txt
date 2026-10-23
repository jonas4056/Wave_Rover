cmake_minimum_required(VERSION 3.8)
project(wave_rover_dashboard)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch web DESTINATION share/${PROJECT_NAME})

ament_package()
