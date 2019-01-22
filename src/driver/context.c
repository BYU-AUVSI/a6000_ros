#include <stdio.h>
#include "gphoto_drv.h"

static void ctx_error_func(GPContext *context, const char *str, void *data) {
        fprintf  (stderr, "\n*** Contexterror ***              \n%s\n",str);
        fflush   (stderr);
}

static void ctx_status_func(GPContext *context, const char *str, void *data) {
        fprintf  (stderr, "%s\n", str);
        fflush   (stderr);
}

GPContext* create_context() {
	GPContext *context;

	/* This is the mandatory part */
	context = gp_context_new();

	/* All the parts below are optional! */
        gp_context_set_error_func (context, ctx_error_func, NULL);
        gp_context_set_status_func (context, ctx_status_func, NULL);
        
	return context;
}

int autodetect(CameraList *list, GPContext *context) {
	gp_list_reset (list);
        return gp_camera_autodetect (list, context);
}

// GP_PORT TYPE
// 0 - GP_PORT_NONE 	
// 1 - GP_PORT_SERIAL
// 2 - GP_PORT_USB
// 3 - GP_PORT_DISK 	
// 4 - GP_PORT_PTPIP 	
// 5 - GP_PORT_USB_DISK_DIRECT 	
// 6 - GP_PORT_USB_SCSI 
void printCameraPortInfo(Camera* camera) {
        GPPortInfo portInfo;
        GPPortType type;
        char* charData;
        int ret = gp_camera_get_port_info(camera, &portInfo);

        if (ret < GP_OK) {
                printf("Failed to get Port Info!\n");
        } else {
                printf("PORT INFO::\n");
                gp_port_info_get_type(portInfo, &type);
                printf("\ttype: %i\n", type);
                gp_port_info_get_name(portInfo, &charData);
                printf("\tname: %s\n", charData);
                gp_port_info_get_path(portInfo, &charData);
                printf("\tpath: %s\n", charData);
        }
}