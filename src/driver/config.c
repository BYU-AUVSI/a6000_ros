#include "gphoto_drv.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * This function looks up a label or key entry of
 * a configuration widget.
 * The functions descend recursively, so you can just
 * specify the last component.
 */

static int _lookup_widget(CameraWidget*widget, const char *key, CameraWidget **child) {
	int ret;
	ret = gp_widget_get_child_by_name (widget, key, child);
	if (ret < GP_OK)
		ret = gp_widget_get_child_by_label (widget, key, child);
	return ret;
}


int get_config_type(Camera *camera, const char *key, GPContext *context) {
 	CameraWidget		*widget = NULL, *child = NULL;
	int					ret;
	CameraWidgetType	type;

	ret = gp_camera_get_single_config (camera, key, &child, context);
	if (ret == GP_OK) {
		if (!child) printf("   child is NULL?\n");
		widget = child;
	} else {
		ret = gp_camera_get_config (camera, &widget, context);
		if (ret < GP_OK) {
			printf("   camera_get_config failed: %d\n", ret);
			return ret;
		}
		ret = _lookup_widget (widget, key, &child);
		if (ret < GP_OK) {
			printf("   lookup widget failed: %d\n", ret);
			gp_widget_free (widget);
			return ret;
		}
	}

	ret = gp_widget_get_type (child, &type);
	if (ret < GP_OK) {
		printf("   Get config type failed: %d\n", ret);
		gp_widget_free(widget);
		return ret;
	}
	gp_widget_free(widget);
	return type;
}

float get_config_value_float(GPContext *context, Camera *camera, const char *key) {
	CameraWidget		*widget = NULL, *child = NULL;
	int					ret;
	float test;

	ret = gp_camera_get_single_config (camera, key, &child, context);
	if (ret == GP_OK) {
		if (!child) printf("   child is NULL?\n");
		widget = child;
	} else {
		ret = gp_camera_get_config (camera, &widget, context);
		if (ret < GP_OK) {
			printf("   camera_get_config failed: %d\n", ret);
			return (float) ret;
		}
		ret = _lookup_widget (widget, key, &child);
		if (ret < GP_OK) {
			printf("   lookup widget failed: %d\n", ret);
			gp_widget_free (widget);
			return (float) ret;
		}
	}

	/* This is the actual query call. Note that we just*/
	ret = gp_widget_get_value(child, &test);
	
	if (ret < GP_OK) {
		printf("   could not query widget float value: %d\n", ret);
		gp_widget_free (widget);
		return (float) ret;
	}
	gp_widget_free (widget);
	return test;
}

int get_config_value_string(GPContext *context, Camera *camera, const char *key, char **val) {
	CameraWidget		*widget = NULL, *child = NULL;
	int					ret;
	char*				str;

	ret = gp_camera_get_single_config (camera, key, &child, context);
	if (ret == GP_OK) {
		if (!child) printf("   child is NULL?\n");
		widget = child;
	} else {
		ret = gp_camera_get_config (camera, &widget, context);
		if (ret < GP_OK) {
			printf("   camera_get_config failed: %d\n", ret);
			return ret;
		}
		ret = _lookup_widget (widget, key, &child);
		if (ret < GP_OK) {
			printf("   lookup widget failed: %d\n", ret);
			goto out;
		}
	}

	/* This is the actual query call. Note that we just*/
	ret = gp_widget_get_value(child, &str);
	
	if (ret < GP_OK) {
		printf("   could not query widget value: %d\n", ret);
		goto out;
	}
	*val = strdup(str);
out:
	gp_widget_free (widget);
	return ret;
}



/* Sets a string configuration value.
 * This can set for:
 *  - A Text widget
 *  - The current selection of a Radio Button choice
 *  - The current selection of a Menu choice
 *
 * Sample (for Canons eg):
 *   get_config_value_string (camera, "owner", &ownerstr, context);
 */
int set_config_value_string(Camera *camera, const char *key, const char *val, GPContext *context) {
	CameraWidget		*widget = NULL, *child = NULL;
	CameraWidgetType	type;
	int			ret;

	ret = gp_camera_get_config (camera, &widget, context);
	if (ret < GP_OK) {
		fprintf (stderr, "camera_get_config failed: %d\n", ret);
		return ret;
	}
	ret = _lookup_widget (widget, key, &child);
	if (ret < GP_OK) {
		fprintf (stderr, "lookup widget failed: %d\n", ret);
		goto out;
	}

	/* This type check is optional, if you know what type the label
	 * has already. If you are not sure, better check. */
	ret = gp_widget_get_type (child, &type);
	if (ret < GP_OK) {
		fprintf (stderr, "widget get type failed: %d\n", ret);
		goto out;
	}
	switch (type) {
        case GP_WIDGET_MENU:
        case GP_WIDGET_RADIO:
        case GP_WIDGET_TEXT:
		break;
	default:
		fprintf (stderr, "widget has bad type %d\n", type);
		ret = GP_ERROR_BAD_PARAMETERS;
		goto out;
	}

	/* This is the actual set call. Note that we keep
	 * ownership of the string and have to free it if necessary.
	 */
	ret = gp_widget_set_value (child, val);
	if (ret < GP_OK) {
		fprintf (stderr, "could not set widget value: %d\n", ret);
		goto out;
	}
	ret = gp_camera_set_single_config (camera, key, child, context);
	if (ret != GP_OK) {
		/* This stores it on the camera again */
		ret = gp_camera_set_config (camera, widget, context);
		if (ret < GP_OK) {
			fprintf (stderr, "camera_set_config failed: %d\n", ret);
			return ret;
		}
	}
out:
	gp_widget_free (widget);
	return ret;
}
