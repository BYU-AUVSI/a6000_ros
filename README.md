# Sony a6000 ROS

This package exposes a Sony a6000 to a ROS network by using [GPhoto2](https://github.com/gphoto/). While this driver was primarily built for the a6000, it should easily extend to other cameras. The ROS node itself is added as a wrapper class to the core driver, meaning that it should be easy to continue using the base driver code without ROS if needed.

## Dependencies

This package requires a number of dependencies that are not installed in a default ROS linux environment. The main dependency is:

- [libgphoto2](https://github.com/gphoto/libgphoto2) (Developed on the 2.5.21 release)

The `install-deps.sh` scripts in the root of the repository will install a verified working release
of libgphoto2. It should work on ARM or x86 linux Ubuntu 16.04. (Currently untested on other versions).

## Setup

Configure your a6000 in 'PC Remote' mode by navigating: `Menu -> Suitcase tab on top right -> Page 4 -> USB Connection -> PC Remote`.

Plug the camera into the computer via USB.

## Build

By default this project's cmake configuration will generate the driver with a ROS wrapper, as well as a ROS-independent `_test` executable. This allows you to easily test driver specific functions without having to worry about ROS configuration.

## Services

You can control certain camera settings through the following ros services:

## Extending to other cameras

First, check if the desired camera is on [gphoto2's list of supported cameras](http://gphoto.org/proj/libgphoto2/support.php).

You'll would need to provide configuration setting details in `camera_config_defs.h` and `camera_config_defs.cpp`. Also note compared to other supported cameras, the a6000's capabilities over gphoto2 are fairly basic. Meaning, that for better supported cameras, there may be better ways of interacting with them, than how this driver does.