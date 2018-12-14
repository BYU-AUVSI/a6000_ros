#include "camera_connector.h"


int main(int argc, char const *argv[]) {
    
    CameraConnector camLink = CameraConnector();
    camLink.blockingConnect();

    char configValue[100];
    int i = 0;
    int numConfigValues;
    string configValueOpts[60];
    printf("lets get a setting or two\n");
    
    // printf("for IMAGE_SIZE its...\n");
    // camLink.getConfigStringValue(A6000Config::IMAGE_SIZE.settingLabel, configValue);
    // printf("   %s\n", configValue);
    // printf("for ISO its...\n");
    // camLink.getConfigStringValue(A6000Config::ISO.settingLabel, configValue);
    // printf("   %s\n", configValue);
    // printf("for WHITE_BALANCE its...\n");
    // camLink.getConfigStringValue(A6000Config::WHITE_BALANCE.settingLabel, configValue);
    // printf("   %s\n", configValue);
    // printf("for EXPOSURE_COMP its...\n");
    // camLink.getConfigStringValue(A6000Config::EXPOSURE_COMP.settingLabel, configValue);
    // printf("   %s\n", configValue);
    // printf("for FLASH_MODE its...\n");
    // camLink.getConfigStringValue(A6000Config::FLASH_MODE.settingLabel, configValue);
    // printf("   %s\n", configValue);
    printf("for F_STOP its...\n");
    camLink.getConfigStringValue(A6000Config::F_STOP.settingLabel, configValue);
    printf("   %s\n", configValue);
    camLink.getConfigOptions(&A6000Config::F_STOP, configValueOpts, &numConfigValues);
    printf("   has %d possible values:\n", numConfigValues);
    while (i < numConfigValues) {
        cout << configValueOpts[i] << endl;
        i++;
    }
    printf("for IMAGE_QUALITY its...\n");
    camLink.getConfigStringValue(A6000Config::IMAGE_QUALITY.settingLabel, configValue);
    printf("   %s\n", configValue);
    i = 0;
    camLink.getConfigOptions(&A6000Config::IMAGE_QUALITY, configValueOpts, &numConfigValues);
    printf("   has %d possible values:\n", numConfigValues);
    while (i < numConfigValues) {
        cout << configValueOpts[i] << endl;
        i++;
    }
    // printf("for FOCUS_MODE its...\n");
    // camLink.getConfigStringValue(A6000Config::FOCUS_MODE.settingLabel, configValue);
    // printf("   %s\n", configValue);
    // printf("for EXP_PROGRAM its...\n");
    // camLink.getConfigStringValue(A6000Config::EXP_PROGRAM.settingLabel, configValue);
    // printf("   %s\n", configValue);
    // printf("for ASPECT_RATIO its...\n");
    // camLink.getConfigStringValue(A6000Config::ASPECT_RATIO.settingLabel, configValue);
    // printf("   %s\n", configValue);
    // printf("for CAPTURE_MODE its...\n");
    // camLink.getConfigStringValue(A6000Config::CAPTURE_MODE.settingLabel, configValue);
    // printf("   %s\n", configValue);
    printf("for SHUTTER_SPEED its...\n");
    camLink.getConfigStringValue(A6000Config::SHUTTER_SPEED.settingLabel, configValue);
    printf("   %s\n", configValue);
    // camLink.getConfigOptions(&A6000Config::SHUTTER_SPEED, configValueOpts, &numConfigValues);
    // i = 0;
    // printf("   has %d possible values:\n", numConfigValues);
    // while (i < numConfigValues) {
    //     cout << configValueOpts[i] << endl;
    //     i++;
    // }

    // printf("for EXPOSURE_METER_MODE its...\n");
    // camLink.getConfigStringValue(A6000Config::EXPOSURE_METER_MODE, configValue);
    // printf("   %s\n", configValue);

    

    // capture multiple images (working)
    char* imgData;
    unsigned long imgSize;
    char name[20];

    for (int i = 0; i < 5; i++) {

        camLink.captureImage((const char**)&imgData, &imgSize);
        sprintf(name, "testeroni-%d.jpg", i);
        camLink.writeImageToFile(name, imgData, imgSize);
        if (imgData != nullptr) {
            printf("size:: %ld\n", imgSize);

        } else {
            printf("errrmmmmmm...");
        }
    }


    printf("did we cap??\n");

    return 0;
}
