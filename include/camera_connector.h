#pragma once
#include "gphoto_drv.h"
#include "exif.h" // easy exif library for image metadata
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

    /**
     * Doesn't exit this function until it has successfully connected to the camera
     * or experienced a critical error. 3 seconds between connection attempts
     * 
     * @return: True when connected to camera, otherwise false.
     */
    bool blockingConnect();

    /**
     * Attempts to connect a single time to a connected camera.
     * Calling
     * 
     * 
     */
    bool attemptConnection();

    bool isConnected();

    /** Capture an image off the camera, and store it on memory.
     * image_data will the image as a byte array (in the case of the a6000,
     * this data array is fully in a jpeg format). Size will return with the 
     * corresponding size of the image.
     * 
     * NOTE: This method assumes serial operation. ie: capture one image, process
     *  it and then capture another image, without ever needing to reference the 
     *  previous image again. It is the responsibility of the user to memcopy
     *  the buffer if they want to access an old image after calling this function
     *  a second time.
     * 
     */
    bool captureImage(const char** image_data, unsigned long* size);

    bool lastImageHasEXIF();

    easyexif::EXIFInfo getExif();

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
    std::string getAllConfigBasicInfo(const vector<ConfigSetting> settings);
    
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

    void connectFromList(CameraList* list);

    GPContext* context = nullptr;
    Camera *camera = nullptr;
    CameraFile *currentFile_ = nullptr;
    easyexif::EXIFInfo exifInfo;
    bool connected_;
    bool autoReconnect_ = true;
    bool hasExif_ = false;
};


