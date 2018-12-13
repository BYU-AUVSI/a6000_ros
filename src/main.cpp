#include "camera_connector.h"

int main(int argc, char const *argv[]) {
    
    printf("waddup\n");

    CameraConnector camLink = CameraConnector();

    camLink.blockingConnect();
    
    printf("did we connect???\n");
    return 0;
}
