cmake_minimum_required(VERSION 3.5)
project(warehouse_amr)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz worlds config maps
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/mission_executor.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
