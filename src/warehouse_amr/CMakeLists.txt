cmake_minimum_required(VERSION 3.5)
project(warehouse_amr)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

install(DIRECTORY urdf launch rviz worlds config maps
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/mission_executor.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_pytest_test(test_mission_executor
    test/test_mission_executor.py
    TIMEOUT 60
  )
endif()

ament_package()