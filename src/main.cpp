#include <stdio.h>

#include "gphoto_drv.h"


int main(int argc, char const *argv[]) {
    CameraList *list;
    printf("waddup\n");

    int ret = gp_list_new (&list);
    GPContext* gpcontext = create_context();


    if (ret < GP_OK) {
        printf("Gphoto's got issues:: %d\n", ret);
        return 1;
    }
    int cameraCount = autodetect(list, gpcontext);
    if (cameraCount < GP_OK) {
        printf("Couldn't detect any cameras:: %d\n", cameraCount);
    }
    return 0;
}
