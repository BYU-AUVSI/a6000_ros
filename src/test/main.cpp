#include "camera_connector.h"

double clock_diff_to_sec(long clock_diff)
{
    return double(clock_diff) / CLOCKS_PER_SEC;
}

/**
 * This just runs though and tests some basic 
 * functionality of the camera_connector code.
 * Namely config getting/setting and image capture
 * and saving
 */
int main(int argc, char const *argv[]) {
    
    CameraConnector camLink = CameraConnector(true);
    camLink.blockingConnect();

    printf("Lets retrieve current and possible values for a setting or two...\n");
    cout << camLink.getConfigInfo(&A6000Config::SHUTTER_SPEED);
    cout << camLink.getConfigInfo(&A6000Config::F_STOP) << endl;
    cout << camLink.getConfigInfo(&A6000Config::IMAGE_QUALITY) << endl;

    usleep(3000000);

    printf("\nLets update some settings\n\n");
    // camLink.setConfigValue(&A6000Config::SHUTTER_SPEED, "1/250"); // this will work
    // camLink.setConfigValue(&A6000Config::F_STOP, "6.3"); // this will work (the image is probably really dark now)
    camLink.setConfigValue(&A6000Config::IMAGE_QUALITY, "RAWWWR"); // this should fail

    // capture multiple images 
    printf("\nCapture time!\n");
    
    char* imgData;
    unsigned long imgSize;
    char name[20];

    for (int i = 0; i < 10; i++) {

        if (camLink.captureImage((const char**)&imgData, &imgSize)) {

            sprintf(name, "test-img-%d.jpg", i);
            camLink.writeImageToFile(name, imgData, imgSize);
        } else {
            printf("Failed to capture image %d\n", i);
        }
        
    }

    // camLink.wrapperTest();

    printf("Did it work??\n");
    return 0;
}
