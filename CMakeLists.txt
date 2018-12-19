# default version of cmake on 16.04 is 3.5.1
cmake_minimum_required(VERSION 2.8.3)
project(a6000_ros_driver)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

# IF(APPLE)
#     set(BREW_INCLUDE_DIR /usr/local/include)
# ENDIF(APPLE)

find_package(Gphoto2 REQUIRED)

file(GLOB src_dir
    "src/driver/*.c"
    "src/*.cpp"
)

message(STATUS ${PROJECT_SOURCE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)

add_executable(${PROJECT_NAME} ${src_dir})
target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include ${Gphoto2_INCLUDE_DIRS}) #
target_link_libraries(${PROJECT_NAME} PRIVATE ${Gphoto2_LIBRARIES})