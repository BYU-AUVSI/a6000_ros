# Sony a6000 ROS

This package exposes a Sony a6000 to a ROS network by using [GPhoto2](https://github.com/gphoto/gphoto2).

## Setup

Configure your a6000 in 'PC Remote' mode by navigating: `Menu -> Suitcase tab on top right -> Page 4 -> USB Connection -> PC Remote`.

Plug the camera into the computer via USB.

You can check that the computer and GPhoto2 properly recognize the camera by opening a terminal and typing `gphoto2 --auto-detect`

## Services

You can control certain camera settings through the following ros services: