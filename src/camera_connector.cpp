#include "camera_connector.h"


CameraConnector::CameraConnector() {
    printf("Startup Camera Connector\n");
}

CameraConnector::~CameraConnector() {
    close();
}


void CameraConnector::close() {
    if (context != nullptr && camera != nullptr) {
        gp_camera_exit(camera, context);
        gp_camera_free(camera);
	    gp_context_unref(context);
    }
}

char* CameraConnector::captureImage(char* image_data, unsigned long* size) {
    if (connected) {    
        int	retval = capture_to_memory(camera, context, (const char**)&image_data, size);

        if (retval < GP_OK) {
            printf("Capture failed, aborting...");
            return nullptr;
        }
    } else {
        printf("ERR: Trying to capture an image without a camera connected!\n");
        return nullptr;
    }
    return image_data;
}

bool CameraConnector::writeImageToFile(const char* file_name, const char* image_data, unsigned long size) {
    int retval;
    FILE* f;

    if (image_data != nullptr) {
        std::cout << "Save image to " << file_name << endl;
        f = fopen(file_name, "wb");
        if (f) {
            retval = fwrite(image_data, size, 1, f);
            if (retval != size) {
                printf("  fwrite size %ld, written %d\n", size, retval);
            }
            fclose(f);
            return true;
        } else {
            std::cout << "  fopen " << file_name << " failed." << endl;
        }
    }
    return false; // we only get here if something failed
}

bool CameraConnector::blockingConnect() {
    if (connected) {
        close(); // close out current context in event we're already connected
    }
    connected = false;
    
    CameraList *list;
    int ret = gp_list_new (&list);


     context = create_context();
 
    if (ret < GP_OK) {
        printf("Gphoto's gave us this error trying to create a context:: %d\n", ret);
        return connected;
    }

    while (!connected) {
        int cameraCount = autodetect(list, context);
        
        if (cameraCount < GP_OK || cameraCount == 0) {
            printf("Couldn't detect any cameras - %d\n", cameraCount);
            usleep(3000000);
        } else  {
            printf("Detected %d camera! Connecting to camera 1\n", cameraCount);
            
            //further connection stuff:

            // PTP driver by default tries to traverse entire camera file system on connect.
            // there's a way to stop it from doing this if it becomes untenable (see libgphoto2's sample-capture)
            gp_camera_new(&camera);
            int initRet = gp_camera_init(camera, context);
            if (initRet < GP_OK) {
                printf("Failed to initialize camera! Error code: %d\n", initRet);
                gp_camera_free(camera); //free camera resources again
                // wait between connection attempts. even if the camera is connected
                // it can take a little bit to initialize, and we dont wanna spit out
                // a whole ton of errors in that time
                usleep(3000000);
                continue; //try again, until the user kills us or we connect
            }

            //print out some basic info about the camera we're connecting to
            CameraText text;
            initRet = gp_camera_get_summary(camera, &text, context);
            if (ret < GP_OK) {
                printf("Failed to retrieve basic summary of camera: %d\n", initRet);
                gp_camera_free(camera);
                usleep(3000000);
                continue;
            }

            // The summary from gp_camera_get_summary has a lot of crap we dont care about
            istringstream summaryStream(text.text);
            string summaryLine;
            int numLines = 0;
            cout << "\nCamera info:" << endl;
            while (numLines < 14 && std::getline(summaryStream, summaryLine)) {
                cout << summaryLine << endl;
                numLines++;
            }

            printf("Connection Successful!\n");
            connected = true;
        }

    }

}