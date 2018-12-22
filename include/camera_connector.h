#pragma once
#include "gphoto_drv.h"
#include "camera_config_defs.h" // defines ConfigSetting struct
#include <stdio.h> // printing
#include <iostream> // more printing /shrug
#include <unistd.h> // usleep
#include <sstream>
// used for listing config settings
#include <vector>
#include <algorithm>
// used for multi-threading / timeouts on functionality
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

#define MAX_CONFIG_VALUE_COUNT 60
using namespace std;

class CameraConnector{

public:
    CameraConnector();
    /**
     * Constructor for the Camera connector
     * 
     * @param autoReconnect: specify whether on a failed capture, we should 
     *              instantly try to connect to the camera again through blockingConnect.
     *              Default is true
     */
    CameraConnector(bool autoReconnect);
    ~CameraConnector();

    bool blockingConnect();

    bool isConnected();

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
     * 
     * Given an array of configuration settings, call #getConfigInfo on
     * all of them and and combine the resulting strings into a single string
     * 
     * @returns string with all config info for the settings in the provided array
     */
    std::string getAllConfigInfo(const vector<ConfigSetting> settings);
    
    std::string getConfigOptionsString(const ConfigSetting* setting);
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
    bool connected_;
    bool autoReconnect_ = true;
};


