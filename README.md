# Sony a6000 ROS

This package exposes a Sony a6000 to a ROS network by using [GPhoto2](https://github.com/gphoto/gphoto2).

## Dependencies

This package requires a number of dependencies that are not installed in a default ROS linux environment. The main dependency is:

- [libgphoto2](https://github.com/gphoto/libgphoto2) (Developed on the 2.5.21 release)

The `install-deps.sh` scripts in the root of the repository will install a verified working release
of libgphoto2. It should work on ARM or x86 linux Ubuntu 16.04. (Currently untested on other versions).

## Setup

Configure your a6000 in 'PC Remote' mode by navigating: `Menu -> Suitcase tab on top right -> Page 4 -> USB Connection -> PC Remote`.

Plug the camera into the computer via USB.

You can check that the computer and GPhoto2 properly recognize the camera by opening a terminal and typing `gphoto2 --auto-detect`

## Services

You can control certain camera settings through the following ros services: