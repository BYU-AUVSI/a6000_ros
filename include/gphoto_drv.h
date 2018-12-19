#ifndef gphoto_drv_H
#define gphoto_drv_H
#include <gphoto2/gphoto2-camera.h>

#ifdef __cplusplus
extern "C" {
#endif

int autodetect(CameraList *list, GPContext *context);
GPContext* create_context(void);
int capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size);

//config gettter functions:
float get_config_value_float(GPContext *context, Camera *camera, const char *key);
int get_config_value_float_range(GPContext* context, Camera* camera, const char* key, float* min, float* max, float* step);
int get_config_value_string(GPContext *context, Camera *camera, const char *key, char **val);
int get_config_value_string_choices(GPContext* context, Camera* camera, const char* key, char* values[], int* numValues);
int get_config_type(GPContext *context, Camera *camera, const char *key);

//config setter functions:
int set_config_value_string(GPContext *context, Camera *camera, const char *key, const char *val);

#ifdef __cplusplus
}
#endif

#endif
