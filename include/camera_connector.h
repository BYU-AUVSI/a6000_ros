#pragma once
#include "gphoto_drv.h"
#include "camera_config_defs.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fcntl.h> //file stuff. dont need?

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
     * Get Config value of the given Setting as a String 
     * 
     * @param 
     * @param value: value of the current setting as a string
     */
    void getConfigStringValue(const char* key, char* value);
    void getConfigOptions(const ConfigSetting* setting, char** values);

    void close();

private:
    GPContext* context = nullptr;
    Camera *camera = nullptr;
    bool connected;
};


