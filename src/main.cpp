#include "camera_connector.h"

int main(int argc, char const *argv[]) {
    
    printf("waddup\n");

    CameraConnector camLink = CameraConnector();

    camLink.blockingConnect();

    camLink.captureImage();

    printf("did we cap??\n");

    return 0;
}
