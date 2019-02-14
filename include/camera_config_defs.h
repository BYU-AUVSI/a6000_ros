/***************************
    Copyright (c) 2018, len0rd
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************/
#pragma once
#include <vector>
#include <string.h>

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

    static const std::vector<ConfigSetting> ALL;

    static const ConfigSetting* getConfigSetting(const char* settingName);
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