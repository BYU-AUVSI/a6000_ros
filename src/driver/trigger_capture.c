/**
 * This code comes from libgphoto2's samples, with a bunch modification done by me
 * Note their license is LGPL, though idk how that applies here
 */
#include "gphoto_drv.h"
#include <stdlib.h> 
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <gphoto2/gphoto2.h>

static int got_path_ = 0;
// CameraFilePath final_path_;
static int captured;

static int wait_event_and_download(GPContext *context, Camera *camera, int waittime, CameraFile* file, const char **buffer, unsigned long int *size) {
	CameraEventType	evtype;
	CameraFilePath	*path_;
	void			*data;
	int				retval;
    struct timeval	start, curtime;

    gettimeofday(&start, NULL);
	data = NULL;
	got_path_ = 0;

	while (!got_path_) {
		unsigned int timediff;

        gettimeofday(&curtime, NULL);
		timediff = ((curtime.tv_sec - start.tv_sec)*1000)+((curtime.tv_usec - start.tv_usec)/1000);
		if (timediff >= waittime) {
			break;
		}

		retval = gp_camera_wait_for_event(camera, waittime - timediff, &evtype, &data, context);
		if (retval != GP_OK) {
			printf("return from waitevent in trigger sample with %d\n", retval);
			return retval;
		}
		switch (evtype) {
            case GP_EVENT_CAPTURE_COMPLETE:
            case GP_EVENT_UNKNOWN:
            case GP_EVENT_TIMEOUT:
                break;
            case GP_EVENT_FOLDER_ADDED:
            case GP_EVENT_FILE_CHANGED:
                free(data);
                break;
            case GP_EVENT_FILE_ADDED:
				path_ = data;
				got_path_ = 1;
                break;
		}
	}
	if (got_path_) {
		retval = gp_file_new(&file);

		printf("   camera getfile of %s\n", path_->name);
		retval = gp_camera_file_get(camera, path_->folder, path_->name, GP_FILE_TYPE_NORMAL, file, context);
		
		if (retval != GP_OK) {
			printf ("   gp_camera_file_get failed: %d\n", retval);
			gp_file_free(file);
			return retval;
		}

		/* buffer is returned as pointer, not as a copy */
		retval = gp_file_get_data_and_size(file, buffer, size);
		if (retval != GP_OK) {
			printf("   gp_file_get_data_and_size failed: %d\n", retval);
			gp_file_free(file);
			return retval;
		}

		retval = gp_camera_file_delete(camera, path_->folder, path_->name, context);
		free(data);
		captured = 1;
	}
	return GP_OK;
}

static double avg_ts(struct timeval* ts1, struct timeval* ts2) {
	// get the average timestamp value between two time structs as a double unix epoch TS
	double avg;
	avg = (ts1->tv_sec + (ts1->tv_usec / 1000000.0)) + (ts2->tv_sec + (ts2->tv_usec / 1000000.0));
	avg /= 2; //resolution error? tests seems to be pretty accurate. could be an issue for very large TS?
	return avg;
}

int trigger_capture_to_memory(GPContext *context, Camera *camera, CameraFile* file, const char** data, unsigned long* size, double* trigger_ts) {
	int		retval;
	captured = 0;
	time_t lastTrigger;
	struct timeval triggerStart, triggerEnd;

	lastTrigger = time(NULL);
	gettimeofday(&triggerStart, NULL);
	retval = gp_camera_trigger_capture(camera, context);
	gettimeofday(&triggerEnd, NULL);
	*trigger_ts = avg_ts(&triggerStart, &triggerEnd);

	if ((retval != GP_OK) && (retval != GP_ERROR) && (retval != GP_ERROR_CAMERA_BUSY)) {
		printf("   triggering capture had error %d\n", retval);
		return retval;
	}
	while (!captured) {
		retval = wait_event_and_download(context, camera, 250, file, data, size);
		if (captured || retval != GP_OK) {
			break;
		}

		if (time(NULL)  > (lastTrigger + 2))  { // retrigger every 2 seconds
			// TODO: if we re-trigger a capture, should we update the returned trigger_ts?
			//		  Need to test more. seems like it'll often end up returning the original 
			// 		  trigger captured image when we do this?
			lastTrigger = time(NULL);
			gettimeofday(&triggerStart, NULL);
			retval = gp_camera_trigger_capture(camera, context);
			gettimeofday(&triggerEnd, NULL);
			if ((retval != GP_OK) && (retval != GP_ERROR) && (retval != GP_ERROR_CAMERA_BUSY)) {
				printf("   triggering capture had error %d\n", retval);
				*trigger_ts = 0;
				break;
			}
			*trigger_ts = avg_ts(&triggerStart, &triggerEnd); // update the trigger timestamp?
		}
	}

	return retval;
}