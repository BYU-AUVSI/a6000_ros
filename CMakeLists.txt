cmake_minimum_required(VERSION 3.7)
project(a6000_ros_driver)


set(CMAKE_CXX_STANDARD 11)

find_package(gphoto2 REQUIRED)

file(GLOB src_dir
    "src/*.cpp"
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)

add_executable(${PROJECT_NAME} ${src_dir})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PRIVATE gphoto2)