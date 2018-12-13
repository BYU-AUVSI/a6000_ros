#include <stdio.h>
#include <string.h>
#include "gphoto_drv.h"

int capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size) {
	int retval;
	CameraFile *file;
	CameraFilePath camera_file_path;

	printf("Capturing...\n");

	/* NOP: This gets overridden in the library to /capt0000.jpg */
	strcpy(camera_file_path.folder, "/");
	strcpy(camera_file_path.name, "foo.jpg");

	retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
    if (retval < GP_OK) {
	    printf("  Error executing capture: %d\n", retval);
        return retval;
    }

	retval = gp_file_new(&file);
	
    if (retval < GP_OK) {
	    printf("  Error creating new gp file: %d\n", retval);
        return retval;
    }

	retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name,
		     GP_FILE_TYPE_NORMAL, file, context);

    if (retval < GP_OK) {
	    printf("  Error get file from camera: %d\n", retval);
        return retval;
    }

	gp_file_get_data_and_size(file, ptr, size);

	retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name,
			context);

    if (retval < GP_OK) {
        //this isnt the end of the world if there's an sd card on board
        // (at least in my ignorance i dont think it's a big deal)
	    printf("  Error trying to delete file off camera: %d\n", retval);
    }
	return 0;
}