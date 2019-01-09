# Sony a6000 ROS

This package exposes a Sony a6000 to a ROS network by using [GPhoto2](https://github.com/gphoto/). While this driver was primarily built for the a6000, it should easily extend to other cameras. The ROS node itself is added as a wrapper class to the core driver, meaning that its also possible to continue using the base driver code without ROS if needed.

The [image_transport](https://wiki.ros.org/image_transport) package is used to publish images onto the ROS network. Note that the base /img topic will reflect whatever quality is set on the camera. ie: if the camera settings are configured to capture 16MP jpg images, these are what will be published on the /img topic, while the /img/compressed topic will be that jpg image, pushed again through image_transport's compressor.

## Dependencies

This package requires a number of dependencies that are not installed in a default ROS linux environment. The main dependency is:

- [libgphoto2](https://github.com/gphoto/libgphoto2) (Developed on the 2.5.21 release)

The `install-deps.sh` script in the root of the repository will install a verified working release
of libgphoto2. It should work on ARM or x86 linux Ubuntu 16.04. (Currently untested on other versions).

## Setup

Configure your a6000 in 'PC Remote' mode by navigating: `Menu -> Suitcase tab on top right -> Page 4 -> USB Connection -> PC Remote`.

To properly get/set settings it also seems to be important to turn off 'Auto Review': `Menu -> Setting Gear Tab -> Page 1 -> Auto Review -> Off`.

Currently we place the camera in as much of a 'manual mode' as possible. Start by setting the top dial to the `M` (manual) setting. Also turn off Pre-AF, to prevent camera capture from freezing up by trying to focus: `Menu -> Setting Gear Tab -> Page 3 -> Pre-AF -> Off`. Set the focus mode to Direct Manual Focus (DMF): `Fn -> Focus Mode -> DMF`. 

Plug the camera into the computer via USB.

## Build

By default this project's cmake configuration will generate the driver with a ROS wrapper, as well as a ROS-independent `_test` executable. This allows you to easily test driver specific functions without having to worry about ROS configuration.

## Services

You can view/control certain camera settings through the following ros services:

### config_list

Gets a list of all configuration settings and their current values. This is useful for getting a quick overview of what settings exist the kind of values they look for. To get more information on the potential values of a setting, use /config_get {setting_name}.

**Example Call:**
```bash
rosservice call /a6000_ros_node/config_list
```

**Example Response:**
```bash
success: True
message: "IMAGE_SIZE = Medium, ISO = 2000, WHITE_BALANCE = Automatic, EXPOSURE_COMP = 0, FLASH_MODE\
  \ = Unknown value 0000, F_STOP = 3.500000, IMAGE_QUALITY = Standard, FOCUS_MODE\
  \ = DMF, EXP_PROGRAM = M, ASPECT_RATIO = 3:2, CAPTURE_MODE = Single Shot, SHUTTER_SPEED\
  \ = 1/25, EXPOSURE_METER_MODE = Average, "
```

### config_get

Get information on a specific setting. When provided a proper setting name, this will retrieve its current value as well as a description of what possible value it can be set to. In the case of the a6000, the possible values for all settings is a string list of exact values that the particular config can be set to.

**Example Call:**
```bash
rosservice call /a6000_ros_node/config_get f_stop
```

**Example Response:**
```bash
exists: True
currentValue: "3.500000"
possibleValues: "{3.5, 4.0, 4.5, 5.0, 5.6, 6.3, 7.1, 8.0, 9.0, 10.0, 11.0, 13.0, 14.0, 16.0, 18.0,\
  \ 20.0, 22.0, }\n"
```

### config_set

Set config option to a specific value. This service will check that the configuration name is valid and that the value is allowed before setting it. Allowed values are defined by the `possibleValues` string returned from a [config_get](#config_get) call. Note for more intensive setting changes (ie: large changes in shutter speed or f-stop), the camera will halt taking images while changing them - which can take 1-3 seconds.

**Example Call:**
```bash
rosservice call /a6000_ros_node/config_set shutter_speed 1/250
```

**Example Response:**
```bash
success: True
message: "Successfully updated camera configuration!"
```

IMPORTANT: For values that are represented as floats (eg: f_stop), you need to make sure its passed as a string in your service call:

```bash
rosservice call /a6000_ros_node/config_set "configName: 'f_stop' 
value: '6.3'"
```

If you try and tab-complete on the service, the template for these values should appear. For whatever reason, the newline between configName and value is required.

## Extending to other cameras

If you're super cool and want to extend this code to work with other cameras, do it!

First, check if the desired camera is on [gphoto2's list of supported cameras](http://gphoto.org/proj/libgphoto2/support.php).

You'll need to provide configuration setting details in `camera_config_defs.h` and `camera_config_defs.cpp`. Also note compared to other supported cameras, the a6000's capabilities using gphoto2 are fairly basic. Meaning, that for better supported cameras, there may be better ways of interacting with them than how this driver does.