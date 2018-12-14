#include "camera_connector.h"


CameraConnector::CameraConnector() {
    printf("Startup Camera Connector\n");
}

CameraConnector::~CameraConnector() {
    close();
}


void CameraConnector::close() {
    if (context != nullptr && camera != nullptr) {
        gp_camera_exit(camera, context);
        gp_camera_free(camera);
	    gp_context_unref(context);
    }
}

std::string CameraConnector::getConfigInfo(const ConfigSetting* setting) {
    char currentValue[50];
    std::string ret = setting->name;
    ret += '\n';

    if (getConfigStringValue(setting->settingLabel, (char*) currentValue)) {
        ret += "Current Setting: ";
        ret += currentValue;
        ret += '\n';
    }

    string configOpts[MAX_CONFIG_VALUE_COUNT];
    int numValues;
    if (getConfigOptions(setting, configOpts, &numValues)) {
        ret += '{';
        for (int i = 0; i < numValues; i++) {
            ret += configOpts[i] + ", ";
        }
        ret += "}\n";
    }

    return ret;
}

bool CameraConnector::getConfigOptions(const ConfigSetting* setting, std::string* values, int* numValues) {
    char* test[MAX_CONFIG_VALUE_COUNT];
    int ret;

    if (connected) {

        if (setting->hasPossibleValues) {
            for (int i = 0; i < setting->numPossibleValues; i++) {
                values[i] = (setting->possibleValues[i]);
            }
            *numValues = setting->numPossibleValues;
            return true;
        } else {
            // get our config type, also make sure the setting currently exists on our camera
            int type = get_config_type(camera, setting->settingLabel, context);
            if (type < GP_OK) {
                printf("Something went wrong while trying to get config type for (%s) - %d\n", setting->settingLabel, type);
                return false;
            }

            // now that we know the type, properly handle it
            switch(type) {
                case GP_WIDGET_MENU:
                case GP_WIDGET_RADIO:
                case GP_WIDGET_TEXT:
                    ret = get_config_value_string_choices(context, camera, setting->settingLabel, test, numValues);
                    if (ret < GP_OK) {
                        printf("ERR: Failed to retrieve config option values: %d\n", ret);
                        return false;
                    }
                    //Todo: this could be better. Taking the char*[] and copying to a string[]
                    for (int i = 0; i < *numValues; i++) {
                        values[i] = (test[i]);
                    }
                    return true;
                //case GP_WIDGET_RANGE: // todo: add support for range min, max, step getting (function exists in config.c)
                default:
                    printf("Sorry, currently this driver only supports a limited number of configuration types\n");
                    std::cout << "If you're trying to dynamically get config options for a RANGE widget, there are methods "
                            << "available in config.c. I just found the results to be unsatisfactory for the camera I "
                            << "was using so instead statically defined the possible options" << endl;;
            }
        }

    } else {
        printf("ERR: Trying to get camera config options without a camera connected!\n");
    }
    return false;

}

bool CameraConnector::getConfigStringValue(const char* key, char* value) {
    char*  rawStr;
    int    ret;
    float  rangeValue;
    
    if (connected) {

        // Get the type of config option that's hoping to be retrieved.
        // If the config option doesnt exist, this method will let us know
        int type = get_config_type(camera, key, context);
        if (type < GP_OK) {
            printf("Something went wrong while trying to get config type for (%s) - %d\n", key, type);
            return false;
        }

        // Now that we know the type, we know how to retrieve it properly
        switch(type) {
            case GP_WIDGET_MENU:
            case GP_WIDGET_RADIO:
            case GP_WIDGET_TEXT:
                ret = get_config_value_string(context, camera, key, &rawStr);
                if (ret < GP_OK) {
                    printf("ERR: Failed to retrieve current config string value %d\n", ret);
                    return false;
                }
                sprintf(value, "%s", rawStr);
                return true;
            case GP_WIDGET_RANGE:
                rangeValue = get_config_value_float(context, camera, key);
                if (rangeValue < GP_OK) {
                    printf("ERR: Failed to retrieve current config float value %2.0f\n", rangeValue);
                    return false;
                }
                sprintf(value, "%f", rangeValue);
                return true;
            default:
                printf("Sorry, currently this driver only supports a limited number of configuration types\n");
        }
        
    } else {
        printf("ERR: Trying to get camera config value without a camera connected!\n");
    }
    return false;
}

bool CameraConnector::captureImage(const char** image_data, unsigned long* size) {
    if (connected) {    
        int	retval = capture_to_memory(camera, context, image_data, size);

        if (retval < GP_OK) {
            printf("Capture failed (%d), aborting...", retval);
            return false;
        }
    } else {
        printf("ERR: Trying to capture an image without a camera connected!\n");
        return false;
    }
    return true;
}

bool CameraConnector::writeImageToFile(const char* file_name, const char* image_data, unsigned long size) {
    int retval;
    FILE* f;

    if (image_data != nullptr) {
        std::cout << "Save image to " << file_name << endl;
        f = fopen(file_name, "wb");
        if (f) {
            retval = fwrite(image_data, size, 1, f);
            if (retval != size) {
                printf("  fwrite size %ld, written %d\n", size, retval);
            }
            fclose(f);
            return true;
        } else {
            std::cout << "  fopen " << file_name << " failed." << endl;
        }
    }
    return false; // we only get here if something failed
}

bool CameraConnector::blockingConnect() {
    if (connected) {
        close(); // close out current context in event we're already connected
    }
    connected = false;
    
    CameraList *list;
    int ret = gp_list_new (&list);


     context = create_context();
 
    if (ret < GP_OK) {
        printf("Gphoto's gave us this error trying to create a context:: %d\n", ret);
        return connected;
    }

    while (!connected) {
        int cameraCount = autodetect(list, context);
        
        if (cameraCount < GP_OK || cameraCount == 0) {
            printf("Couldn't detect any cameras - %d\n", cameraCount);
            usleep(3000000);
        } else  {
            printf("Detected %d camera! Connecting to camera 1\n", cameraCount);
            
            //further connection stuff:

            // PTP driver by default tries to traverse entire camera file system on connect.
            // there's a way to stop it from doing this if it becomes untenable (see libgphoto2's sample-capture)
            gp_camera_new(&camera);
            int initRet = gp_camera_init(camera, context);
            if (initRet < GP_OK) {
                printf("Failed to initialize camera! Error code: %d\n", initRet);
                gp_camera_free(camera); //free camera resources again
                // wait between connection attempts. even if the camera is connected
                // it can take a little bit to initialize, and we dont wanna spit out
                // a whole ton of errors in that time
                usleep(3000000);
                continue; //try again, until the user kills us or we connect
            }

            //print out some basic info about the camera we're connecting to
            CameraText text;
            initRet = gp_camera_get_summary(camera, &text, context);
            if (ret < GP_OK) {
                printf("Failed to retrieve basic summary of camera: %d\n", initRet);
                gp_camera_free(camera);
                usleep(3000000);
                continue;
            }

            // The summary from gp_camera_get_summary has a lot of crap we dont care about
            istringstream summaryStream(text.text);
            string summaryLine;
            int numLines = 0;
            cout << "\nCamera info:" << endl;
            while (numLines < 14 && std::getline(summaryStream, summaryLine)) {
                cout << summaryLine << endl;
                numLines++;
            }

            printf("Connection Successful!\n");
            connected = true;
        }

    }

}