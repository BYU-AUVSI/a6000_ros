#pragma once
#include "gphoto_drv.h"
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

    char* captureImage(char* image_data, unsigned long* size);
    bool writeImageToFile(const char* file_name, const char* image_data, unsigned long size);

    void close();

private:
    GPContext* context = nullptr;
    Camera *camera = nullptr;
    bool connected;
};