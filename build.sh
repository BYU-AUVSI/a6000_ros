
mkdir -p build
cd build
cmake --build ../ --config Debug --target a6000_ros_driver -- -j 6
make