#ifndef gphoto_drv_H
#define gphoto_drv_H
#include <gphoto2/gphoto2-camera.h>

#ifdef __cplusplus
extern "C" {
#endif

int autodetect(CameraList *list, GPContext *context);
GPContext* create_context(void);
int capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size);

#ifdef __cplusplus
}
#endif

#endif
