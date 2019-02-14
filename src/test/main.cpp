/***************************
    Copyright (c) 2018, len0rd
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************/
#include "camera_connector.h"

double clock_diff_to_sec(long clock_diff)
{
    return double(clock_diff) / CLOCKS_PER_SEC;
}

/**
 * This just runs though and tests some basic 
 * functionality of the camera_connector code.
 * Namely config getting/setting and image capture
 * and saving
 */
int main(int argc, char const *argv[]) {
    
    CameraConnector camLink = CameraConnector(true);
    camLink.blockingConnect();

    // printf("Lets retrieve current and possible values for a setting or two...\n");
    // cout << camLink.getConfigInfo(&A6000Config::SHUTTER_SPEED);
    // cout << camLink.getConfigInfo(&A6000Config::F_STOP) << endl;
    // cout << camLink.getConfigInfo(&A6000Config::IMAGE_QUALITY) << endl;

    // usleep(3000000);

    // printf("\nLets update some settings\n\n");
    // // camLink.setConfigValue(&A6000Config::SHUTTER_SPEED, "1/250"); // this will work
    // // camLink.setConfigValue(&A6000Config::F_STOP, "6.3"); // this will work (the image is probably really dark now)
    // camLink.setConfigValue(&A6000Config::IMAGE_QUALITY, "RAWWWR"); // this should fail

    // capture multiple images 
    printf("\nCapture time!\n");
    
    char* imgData;
    unsigned long imgSize;
    char name[20];

    for (int i = 0; i < 10; i++) {

        if (camLink.captureImage((const char**)&imgData, &imgSize)) {

            sprintf(name, "test-img-%d.jpg", i);
            camLink.writeImageToFile(name, imgData, imgSize);
        } else {
            printf("Failed to capture image %d\n", i);
        }
        
    }

    printf("Did it work??\n");
    return 0;
}
