#pragma once
#include "gphoto_drv.h"
#include "camera_config_defs.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <fcntl.h> //file stuff. dont need?

#define MAX_CONFIG_VALUE_COUNT 60
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
     * @returns: A string with the relevant information
     *           Formatted as:
     *            "Setting Name\n
     *             CurrentValue\n
     *            {PossibleValue1, PossibleValue2...}"
     */
    std::string getConfigInfo(const ConfigSetting* setting);
    
    /**
     * Get Config value of the given Setting as a String 
     * 
     * @param setting: The setting to retrieve the current value for
     * @param value: Will hold the current value for the setting if successful
     * @returns: Success status. True if successfully retrieved current config value, otherwise false.
     */
    bool getConfigStringValue(const ConfigSetting* setting, char* value);
    
    /**
     * Get the potential values for a setting as an array of strings
     * 
     * @param setting: setting to get the possible options for
     * @param values: String[] to store the possible values in
     * @param numValues: Int pointer which on return will hold the total
     *                   number of possible values for the setting (if successful)
     * @returns: Success status. True if successfully retrieved config options, otherwise false
     */
    bool getConfigOptions(const ConfigSetting* setting, vector<string>* values, int* numValues);

    bool setConfigValue(const ConfigSetting* setting, std::string value);

    void close();

private:
    GPContext* context = nullptr;
    Camera *camera = nullptr;
    bool connected;
};


