#include "gphoto_drv.h"
#include <stdlib.h> 
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <gphoto2/gphoto2.h>

static struct queue_entry {
	CameraFilePath	path;
	int offset;
} *queue = NULL;
static int numFileInQueue = 0;

static int captured;

/**
 * This code comes from libgphoto2's samples, with some modification done by me
 */
static int wait_event_and_download(GPContext *context, Camera *camera, int waittime, const char **buffer, unsigned long int *size) {
	CameraEventType	evtype;
	CameraFilePath	*path;
	void			*data;
	int				retval;
    struct timeval	start, curtime;

    gettimeofday(&start, NULL);
	data = NULL;
	if (numFileInQueue)
		waittime = 10; /* just drain the event queue */

	while (1) {
		unsigned int timediff;

        gettimeofday(&curtime, NULL);

		timediff = ((curtime.tv_sec - start.tv_sec)*1000)+((curtime.tv_usec - start.tv_usec)/1000);
		if (timediff >= waittime) 
			break;

		retval = gp_camera_wait_for_event(camera, waittime - timediff, &evtype, &data, context);
		if (retval != GP_OK) {
			printf("return from waitevent in trigger sample with %d\n", retval);
			return retval;
		}
		path = data;
		switch (evtype) {
            case GP_EVENT_CAPTURE_COMPLETE:
                printf("wait for event CAPTURE_COMPLETE\n");
                break;
            case GP_EVENT_UNKNOWN:
            case GP_EVENT_TIMEOUT:
                break;
            case GP_EVENT_FOLDER_ADDED:
                printf("wait for event FOLDER_ADDED\n");
                free(data);
                break;
            case GP_EVENT_FILE_CHANGED:
                printf("wait for event FILE_CHANGED\n");
                free(data);
                break;
            case GP_EVENT_FILE_ADDED:
                printf("   File added to queue\n");
                if (numFileInQueue) {
                    struct queue_entry *q;
                    q = realloc(queue, sizeof(struct queue_entry)*(numFileInQueue+1));
                    if (!q) return GP_ERROR_NO_MEMORY;
                    queue = q;
                } else {
                    queue = malloc (sizeof(struct queue_entry));
                    if (!queue) return GP_ERROR_NO_MEMORY;
                }
                memcpy(&queue[numFileInQueue].path, path, sizeof(CameraFilePath));
                queue[numFileInQueue].offset = 0;
                numFileInQueue++;
                free(data);
                break;
		}
	}
	if (numFileInQueue) {
		int			fd;
		struct stat	stbuf;
		CameraFile	*file;

		retval = gp_file_new(&file);

		printf("   camera getfile of %s\n", queue[0].path.name);
		retval = gp_camera_file_get(camera, queue[0].path.folder, queue[0].path.name,
			GP_FILE_TYPE_NORMAL, file, context);
		
		if (retval != GP_OK) {
			printf ("   gp_camera_file_get failed: %d\n", retval);
			gp_file_free(file);
			return retval;
		}

		/* buffer is returned as pointer, not as a copy */
		retval = gp_file_get_data_and_size(file, buffer, size);

		if (retval != GP_OK) {
			printf("   gp_file_get_data_and_size failed: %d\n", retval);
			gp_file_free (file);
			return retval;
		}

		retval = gp_camera_file_delete(camera, queue[0].path.folder, queue[0].path.name, context);
		memmove(&queue[0],&queue[1],sizeof(queue[0])*(numFileInQueue-1));
		numFileInQueue--;
		captured = 1;
	}
	return GP_OK;
}

int trigger_capture_to_memory(GPContext *context, Camera *camera, const char** data, unsigned long* size) {
	int		retval;
	captured = 0;

	while (!captured) {
		if ((time(NULL) & 1) == 1)  {
			printf("   triggering capture\n");
			retval = gp_camera_trigger_capture(camera, context);
			if ((retval != GP_OK) && (retval != GP_ERROR) && (retval != GP_ERROR_CAMERA_BUSY)) {
				printf("   triggering capture had error %d\n", retval);
				break;
			}
		}
		retval = wait_event_and_download(context, camera, 100, data, size);
		if (retval != GP_OK)
			break;

	}

	return retval;
}