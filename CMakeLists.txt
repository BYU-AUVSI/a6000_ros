# default version of cmake on 16.04 is 3.5.1
cmake_minimum_required(VERSION 2.8.3)
project(a6000_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall") # we should be warning free
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

find_package(Gphoto2 REQUIRED) # camera driver base

# catkin specific stuff (can be removed if you dont wanna deal)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  sensor_msgs
  std_srvs
  message_generation
)

add_service_files(#DIRECTORY srv FILES
    FILES
    ConfigGet.srv
    ConfigSet.srv
)

generate_messages()

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp image_transport sensor_msgs cv_bridge
)

file(GLOB src_dir
    "src/driver/*.c"
    "src/*.cpp"
)

message(STATUS ${PROJECT_SOURCE_DIR})

# test exe, independent of ROS
add_executable(${PROJECT_NAME}_test ${src_dir} src/test/main.cpp)
target_include_directories(${PROJECT_NAME}_test PUBLIC  ${PROJECT_SOURCE_DIR}/include ${Gphoto2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_test PRIVATE ${Gphoto2_LIBRARIES})

# the main ros driver
add_executable(${PROJECT_NAME}_node ${src_dir} src/ros/camera_node.cpp src/ros/camera_ros.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include ${Gphoto2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${Gphoto2_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)