#ifndef gphoto_drv_H
#define gphoto_drv_H
#include <gphoto2/gphoto2-camera.h>

#ifdef __cplusplus
extern "C" {
#endif

int autodetect(CameraList *list, GPContext *context);
int open_camera(Camera ** camera, const char *model, const char *port, GPContext *context);
GPContext* create_context(void);

#ifdef __cplusplus
}
#endif

#endif
