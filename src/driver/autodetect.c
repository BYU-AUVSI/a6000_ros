#include <gphoto2/gphoto2-camera.h>

#include "gphoto_drv.h"

/*
 * This detects all currently attached cameras and returns
 * them in a list. It avoids the generic usb: entry.
 *
 * This function does not open nor initialize the cameras yet.
 */
int autodetect (CameraList *list, GPContext *context) {
	gp_list_reset (list);
        return gp_camera_autodetect (list, context);
}
