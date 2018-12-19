#include "camera_connector.h"


int main(int argc, char const *argv[]) {
    
    CameraConnector camLink = CameraConnector();
    camLink.blockingConnect();

    printf("lets get a setting or two\n");

    cout << camLink.getConfigInfo(&A6000Config::F_STOP) << endl;

    cout << camLink.getConfigInfo(&A6000Config::IMAGE_QUALITY) << endl;

    cout << camLink.getConfigInfo(&A6000Config::SHUTTER_SPEED);

    usleep(3000000);
    printf("WE GONNA DO ITTTTT\n\n");

    camLink.setConfigValue(&A6000Config::SHUTTER_SPEED, "1/4000");
    camLink.setConfigValue(&A6000Config::IMAGE_QUALITY, "RAWWWR");

    // capture multiple images (working)
    // char* imgData;
    // unsigned long imgSize;
    // char name[20];

    // for (int i = 0; i < 5; i++) {

    //     camLink.captureImage((const char**)&imgData, &imgSize);
    //     sprintf(name, "testeroni-%d.jpg", i);
    //     camLink.writeImageToFile(name, imgData, imgSize);
    //     if (imgData != nullptr) {
    //         printf("size:: %ld\n", imgSize);

    //     } else {
    //         printf("errrmmmmmm...");
    //     }
    // }


    printf("did it work??\n");

    return 0;
}
