# this script is particularly useful for building the driver in a non-catkin
# environment. Which would require you to remove all the caktin/ROS junk
# from the CMakeList first
mkdir -p build
cd build
cmake -D BUILD_TESTS=ON --config Debug --target a6000_ros_driver -- -j 6 ../
make