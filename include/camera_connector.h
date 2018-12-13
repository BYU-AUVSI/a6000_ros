#pragma once
#include "gphoto_drv.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>

using namespace std;

class CameraConnector{

public:
    CameraConnector();
    ~CameraConnector();

    bool blockingConnect();
    void close();

private:

    

    GPContext* context = nullptr;
    Camera *camera = nullptr;
};