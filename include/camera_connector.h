#pragma once
#include "gphoto_drv.h"
#include "camera_config_defs.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fcntl.h> //file stuff. dont need?

#define MAX_CONFIG_VALUE_COUNT 40

using namespace std;

class CameraConnector{

public:
    CameraConnector();
    ~CameraConnector();

    bool blockingConnect();

    bool captureImage(const char** image_data, unsigned long* size);
    bool writeImageToFile(const char* file_name, const char* image_data, unsigned long size);

    // Config Options:

    /**
     * Top level function to get all relevant info on a camera config setting
     * as a single string. 
     * 
     * @param setting: Pointer to the configuration setting to get info on 
     * @param info: The String to be populated with the relevant information
     *              Formatted as:
     *                  "Setting Name\n
     *                   CurrentValue\n
     *                   {PossibleValue1, PossibleValue2...}"
     */
    void getConfigInfo(const ConfigSetting* setting, char* info);
    /**
     * Get Config value of the given Setting as a String 
     * 
     * @param 
     * @param value: value of the current setting as a string
     */
    void getConfigStringValue(const char* key, char* value);
    void getConfigOptions(const ConfigSetting* setting, std::string* values, int* numValues);

    void close();

private:
    GPContext* context = nullptr;
    Camera *camera = nullptr;
    bool connected;
};


