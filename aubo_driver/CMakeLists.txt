cmake_minimum_required(VERSION 2.8.3)
project(aubo_driver)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
  aubo_msgs
  industrial_msgs
  actionlib
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs roscpp std_msgs aubo_msgs actionlib
)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(lib ${catkin_INCLUDE_DIRS})

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    link_directories(${catkin_LIB_DIRS}  lib/lib64)
else()
    link_directories(${catkin_LIB_DIRS}  lib/lib32)
endif()

add_executable(aubo_driver src/aubo_driver.cpp)

target_link_libraries(aubo_driver ${catkin_LIBRARIES} libauborobotcontroller.so)
add_dependencies(aubo_driver ${PROJECT_NAME}_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
