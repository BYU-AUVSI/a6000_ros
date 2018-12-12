cmake_minimum_required(VERSION 3.7)
project(a6000_ros_driver)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

# IF(APPLE)
#     set(BREW_INCLUDE_DIR /usr/local/include)
# ENDIF(APPLE)

find_package(Gphoto2 REQUIRED)

file(GLOB src_dir
    "src/*.cpp"
)

file(GLOB lib_dir
    "lib/*.c"
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)

add_executable(${PROJECT_NAME} ${src_dir} ${lib_dir})
target_include_directories(${PROJECT_NAME} PUBLIC include/ lib/ ${Gphoto2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${Gphoto2_LIBRARIES})