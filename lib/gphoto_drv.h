#ifndef __GPHOTO_DRV_H
#define __GPHOTO_DRV_H
#include <gphoto2/gphoto2-camera.h>

extern int autodetect(CameraList *list, GPContext *context);
extern int open_camera(Camera ** camera, const char *model, const char *port, GPContext *context);
extern GPContext* create_context(void);

extern int get_config_value_string(Camera *, const char *, char **, GPContext *);
extern int set_config_value_string(Camera *, const char *, const char *, GPContext *);
int canon_enable_capture(Camera *camera, int onoff, GPContext *context);

extern int camera_auto_focus(Camera *list, GPContext *context, int onoff);
extern int camera_eosviewfinder(Camera *list, GPContext *context, int onoff);
extern int camera_manual_focus(Camera *list, int tgt, GPContext *context);

#if !defined (O_BINARY)
	/*To have portable binary open() on *nix and on Windows */
	#define O_BINARY 0
#endif

#endif
