# Sony a6000 ROS

This package exposes a Sony a6000 to a ROS network by using [GPhoto2](https://github.com/gphoto/). While this driver was primarily build for the a6000, it should easily extend to other cameras. For a different camera, you would need to provide config setting details in `camera_config_defs.h` and `camera_config_defs.cpp`. The ROS node itself is added as a wrapper class to the core driver, meaning that it should be easy to continue using the base driver code without ROS if needed.

## Dependencies

This package requires a number of dependencies that are not installed in a default ROS linux environment. The main dependency is:

- [libgphoto2](https://github.com/gphoto/libgphoto2) (Developed on the 2.5.21 release)

The `install-deps.sh` scripts in the root of the repository will install a verified working release
of libgphoto2. It should work on ARM or x86 linux Ubuntu 16.04. (Currently untested on other versions).

## Setup

Configure your a6000 in 'PC Remote' mode by navigating: `Menu -> Suitcase tab on top right -> Page 4 -> USB Connection -> PC Remote`.

Plug the camera into the computer via USB.

You can check that the computer and GPhoto2 properly recognize the camera by opening a terminal and typing `gphoto2 --auto-detect`

## Build

Building the project is easy. You can build it with or without a catkin_ws.

For a catkin workspace, simply place this package in the src directory of the workspace, and call catkin_make. Assuming you installed the [above](#dependencies) correctly, it should work!

If you want to build everything independently of ROS, you can call `./build.sh` in the root of this repository to build the ROS-independent test code.

## Services

You can control certain camera settings through the following ros services: