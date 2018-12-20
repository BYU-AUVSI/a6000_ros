
typedef struct ConfigSetting {
    const char*  name;  // nice name we give a setting for outside access
    const char*  settingLabel; // setting label in the driver
    bool   hasPossibleValues; // boolean as to whether we manually specified all possible values
    const char** possibleValues; // all possible values, if they needed to be manually specified
    const int numPossibleValues;
} ConfigSetting;


class A6000Config {
public:
    //I got all these possible settings by running: gphoto2 --list-config
    static const ConfigSetting IMAGE_SIZE;
    static const ConfigSetting ISO;
    // static const ConfigSetting COLOR_TEMP; // I have issues with this one and it doesnt seem too important
    static const ConfigSetting WHITE_BALANCE;
    static const ConfigSetting EXPOSURE_COMP;
    static const ConfigSetting FLASH_MODE;
    static const ConfigSetting F_STOP;
    static const ConfigSetting IMAGE_QUALITY;
    static const ConfigSetting FOCUS_MODE;
    static const ConfigSetting EXP_PROGRAM;
    static const ConfigSetting ASPECT_RATIO;
    static const ConfigSetting CAPTURE_MODE;
    static const ConfigSetting SHUTTER_SPEED;
    static const ConfigSetting EXPOSURE_METER_MODE;
};

// This is the relevant info that `gphoto2 --list-config` returns
// for the driver, you only need the last setting name ie: 'shutterspeed'
// /main/actions/bulb                                                             
// /main/actions/movie
// /main/status/serialnumber
// /main/status/manufacturer
// /main/status/cameramodel
// /main/status/deviceversion
// /main/status/vendorextension
// /main/imgsettings/imagesize
// /main/imgsettings/iso
// /main/imgsettings/colortemperature
// /main/imgsettings/whitebalance
// /main/capturesettings/exposurecompensation
// /main/capturesettings/flashmode
// /main/capturesettings/f-number
// /main/capturesettings/imagequality
// /main/capturesettings/focusmode
// /main/capturesettings/expprogram
// /main/capturesettings/aspectratio
// /main/capturesettings/capturemode
// /main/capturesettings/exposuremetermode
// /main/capturesettings/shutterspeed