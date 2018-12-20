# default version of cmake on 16.04 is 3.5.1
cmake_minimum_required(VERSION 2.8.3)
project(a6000_ros_driver)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

find_package(Gphoto2 REQUIRED) # camera driver base
find_package(Threads REQUIRED) # for multi threading

file(GLOB src_dir
    "src/driver/*.c"
    "src/*.cpp"
)

message(STATUS ${PROJECT_SOURCE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)

# test exe, independent of ROS
add_executable(${PROJECT_NAME}_test ${src_dir} src/test/main.cpp)
target_include_directories(${PROJECT_NAME}_test PUBLIC  ${PROJECT_SOURCE_DIR}/include ${Gphoto2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_test PRIVATE ${Gphoto2_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})


# the main ros driver
add_executable(${PROJECT_NAME} ${src_dir} src/ros/camera_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include ${Gphoto2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${Gphoto2_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})