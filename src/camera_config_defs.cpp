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
#include "camera_config_defs.h"

const char* shutterSpeeds[55] = {"Bulb", "300/10", "250/10", "200/10", "150/10", "130/10", "100/10", "80/10", "60/10", "50/10", "40/10", "32/10", "25/10", //13
                                "20/10", "16/10", "13/10", "10/10", "8/10", "6/10", "5/10", "4/10", "1/3", "1/4", "1/5", "1/6", "1/8", "1/10", "1/13", "1/15", "1/20", "1/25", "1/30", "1/40", //20
                                "1/50", "1/60", "1/80", "1/100", "1/125", "1/160", "1/200", "1/250", "1/320", "1/400", "1/500", "1/640", "1/800", "1/1000", "1/1250", "1/1600", "1/2000", "1/2500", //18
                                "1/3200", "1/4000"}; //2
const char* fStops[18] = {"3.5", "4.0", "4.5", "5.0", "5.6", "6.3", "7.1", "8.0", "9.0", "10.0", "11.0", "13.0", "14.0", "16.0", "18.0", "20.0", "22.0"};

//define all constants!!
const ConfigSetting A6000Config::IMAGE_SIZE      = {"IMAGE_SIZE", "imagesize", false, nullptr, 0};
const ConfigSetting A6000Config::ISO             = {"ISO", "iso", false, nullptr, 0};
// const ConfigSetting A6000Config::COLOR_TEMP      = {"COLOR_TEMP"}"colortemperature";
const ConfigSetting A6000Config::WHITE_BALANCE   = {"WHITE_BALANCE", "whitebalance", false, nullptr, 0};
const ConfigSetting A6000Config::EXPOSURE_COMP   = {"EXPOSURE_COMP", "exposurecompensation", false, nullptr, 0};
const ConfigSetting A6000Config::FLASH_MODE      = {"FLASH_MODE", "flashmode", false, nullptr, 0};
const ConfigSetting A6000Config::F_STOP          = {"F_STOP", "f-number", true, fStops, 17};
const ConfigSetting A6000Config::IMAGE_QUALITY   = {"IMAGE_QUALITY", "imagequality", false, nullptr, 0};
const ConfigSetting A6000Config::FOCUS_MODE      = {"FOCUS_MODE", "focusmode", false, nullptr, 0};
const ConfigSetting A6000Config::EXP_PROGRAM     = {"EXP_PROGRAM", "expprogram", false, nullptr, 0}; //lets you select from camera presets. actually pretty useful
const ConfigSetting A6000Config::ASPECT_RATIO    = {"ASPECT_RATIO", "aspectratio", false, nullptr, 0};
const ConfigSetting A6000Config::CAPTURE_MODE    = {"CAPTURE_MODE", "capturemode", false, nullptr, 0};
const ConfigSetting A6000Config::SHUTTER_SPEED   = {"SHUTTER_SPEED", "shutterspeed", true, shutterSpeeds, 53};
const ConfigSetting A6000Config::EXPOSURE_METER_MODE = {"EXPOSURE_METER_MODE", "exposuremetermode", false, nullptr, 0};
const std::vector<ConfigSetting> A6000Config::ALL {A6000Config::IMAGE_SIZE, A6000Config::ISO, A6000Config::WHITE_BALANCE, 
                                            A6000Config::EXPOSURE_COMP, A6000Config::FLASH_MODE,
                                            A6000Config::F_STOP, A6000Config::IMAGE_QUALITY, A6000Config::FOCUS_MODE, 
                                            A6000Config::EXP_PROGRAM, A6000Config::ASPECT_RATIO, A6000Config::CAPTURE_MODE, 
                                            A6000Config::SHUTTER_SPEED, A6000Config::EXPOSURE_METER_MODE};


const ConfigSetting* A6000Config::getConfigSetting(const char* settingName) {
    for (unsigned int i = 0; i < ALL.size(); i++) {
        if (strcmp(settingName, ALL.at(i).name) == 0) {
            return &ALL.at(i);
        }
    }
    return nullptr;
}