#include "camera_connector.h"


int main(int argc, char const *argv[]) {
    
    printf("waddup\n");
    CameraConnector camLink = CameraConnector();
    camLink.blockingConnect();

    char* imgData;
    unsigned long imgSize;

    imgData = camLink.captureImage(imgData, &imgSize);

    if (imgData != nullptr) {
        printf("size:: %ld\n", imgSize);

        camLink.writeImageToFile("testeroni.jpg", imgData, imgSize);
    } else {
        printf("errrmmmmmm...");
    }

    printf("did we cap??\n");

    return 0;
}
