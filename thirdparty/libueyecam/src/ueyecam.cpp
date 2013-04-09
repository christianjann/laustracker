/*
 * Copyright (c) 2013, Chemnitz University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "../include/ueyecam.h"
#include "string.h"
#include <iostream>
#include <time.h>
#include <opencv2/highgui/highgui.hpp>  // für imwrite
#include <stdio.h>

// is_SaveParameters() and is_LoadParameters() gets replaced by
// is_ParameterSet() but we're still using them
#include <ueye_deprecated.h>

using namespace std;
using namespace cv;

UEyeCam::UEyeCam() {
    ueye_configIni = "/cam/set1";
    use_aoi = false;
    aoi_img_width = -1;
    aoi_img_height = -1;
    aoi_img_left = -1;
    aoi_img_top = -1;

}

UEyeCam::~UEyeCam() {
    cout << "UEyeCam Dekonstruktor" << endl;

    // free old image mem.
    // is_FreeImageMem(ueyeCameraID, ueye_pcImageMemory, ueye_lMemoryId);
    // ExitCamera, sonst bekommen wir einen Segmentation fault
    is_ExitCamera(cameraHandle);

}

void UEyeCam::setConfigurationFile(std::string configFile) {
    ueye_configIni = configFile;
}

bool UEyeCam::loadConfiguration() {

    unsigned long int ueye_ret;

    if (is_CameraStatus(this->cameraHandle,
                        IS_PARAMETER_SET_1, ueye_ret) != IS_SUCCESS) {
        cerr << "Could not get camera status information!" << endl;
    }
    //else {
    //  // If IS_PARAMETER_SET_1 is available -> use it!
    //  if (ueye_ret == TRUE) ueye_configIni = "/cam/set1";
    //}

    // Start the camera configuration
    // First try to load the parameters stored in the camera if this fails
    // because is_LoadParameters does not work reliable try to load
    // the configuration from the *.ini file
    // There may be a upstream bug in ueye_api because the exposure time
    // gets not loaded

    cout << "Try loading " << this->ueye_configIni.c_str() << endl;

    // Try to load the configuration file
    if (is_LoadParameters(this->cameraHandle,
                          ueye_configIni.c_str()) != IS_SUCCESS) {
        cerr << "Could not load camera configuration from: "
             << ueye_configIni.c_str() << endl;

        // Try to load the default *.INI file
        ueye_configIni = string(ueye_sInfo.strSensorName) + "_conf.ini";
        cerr << "Trying sensor specific configuration file: "
             << ueye_configIni.c_str() << endl;

        if (is_LoadParameters(this->cameraHandle,
                              ueye_configIni.c_str()) != IS_SUCCESS) {
            cerr << "Could not load camera configuration. Using default."
                 << endl;
            return false;
        } else {
            cout << "Camera configuration loaded from: "
                 << ueye_configIni.c_str() << endl;
            return true;
        }
    } else {
        cout << "Camera configuration loaded from: "
             << ueye_configIni.c_str() << endl;
        return true;
    }
}

UEYE_CAMERA_LIST* getUEyeCameraList() {

    UEYE_CAMERA_LIST* ueyeCameraList;
    int numberOfCameras;

    if (is_GetNumberOfCameras(&numberOfCameras) != IS_SUCCESS) {
        return NULL;
    }

    if (numberOfCameras == 0) {
        return NULL;
    }

    // This is pretty ugly but UEYE_CAMERA_LIST only contains an array
    // UEYE_CAMERA_INFO uci[1].
    // That way one can not get information about more than one camera.
    // A struct member UEYE_CAMERA_INFO *uci would solve this issue.
    ueyeCameraList = (UEYE_CAMERA_LIST*) new char[sizeof(ULONG) +
                     numberOfCameras * sizeof(UEYE_CAMERA_INFO)];

    ueyeCameraList->dwCount = numberOfCameras;

    if (is_GetCameraList(ueyeCameraList) != IS_SUCCESS) {
        return NULL;
    } else {
        return ueyeCameraList;
    }

}

int UEyeCam::getEffectiveImageResolution(int *width, int *height) {

    // For compatibility with new versions of libueyeapi,
    // function is_setAOI was replaced by is_AOI.
    IS_RECT rectAOI;
    if (is_AOI(this->cameraHandle, IS_AOI_IMAGE_GET_AOI,
               (void*)&rectAOI, sizeof(rectAOI)) != IS_SUCCESS) {
        cerr << "is_AOI raised error" << endl;
        return UEYE_FAILED;
    } else {
        *width = (int)rectAOI.s32Width;
        *height = (int)rectAOI.s32Height;
        return UEYE_SUCCESS;
    }

}

int UEyeCam::setAOI(int width, int height, int left, int top) {

    use_aoi = true;
    aoi_img_width = width;
    aoi_img_height = height;
    aoi_img_left = left;
    aoi_img_top = top;

    return UEYE_SUCCESS;
}

int UEyeCam::initCam(HIDS ueyeCameraID) {

    this->ueyeCameraID = ueyeCameraID;

    cout << "Try to open camera " << this->ueyeCameraID << endl;

    if (is_InitCamera(&this->ueyeCameraID, NULL) != IS_SUCCESS) {
        cerr << "Could not open Camera!" << endl;
        return UEYE_FAILED;
    }

    // UGLY: is_InitCamera() expects the camera id as first parameter.
    // If it successfully opens the camera the parameter contains the handle
    // associated with the camera.

    this->cameraHandle = this->ueyeCameraID;
    this->ueyeCameraID = ueyeCameraID;

    if (is_GetSensorInfo(this->cameraHandle, &ueye_sInfo) != IS_SUCCESS) {
        cerr << "Could not get sensor information!" << endl;
        return UEYE_FAILED;
    }

    cout << "Connection to Camera established." << endl;
    cout << "SensorName = " << ueye_sInfo.strSensorName << endl;
    cout << " MaxWidth: " << ueye_sInfo.nMaxWidth << endl;
    cout << " MaxHeight: " << ueye_sInfo.nMaxHeight << endl;

    // load the configuration file
    loadConfiguration();

    ueye_nColorMode = IS_CM_BGR8_PACKED;         // take color images
    ueye_nBitsPerPixel = 24;

    // Switch to color mode
    if (is_SetColorMode(this->cameraHandle, ueye_nColorMode) != IS_SUCCESS) {
        cerr << "Could not set color mode!" << endl;
        return UEYE_FAILED;
    }

    if (getEffectiveImageResolution(&ueye_img_width,
                                    &ueye_img_height) != UEYE_SUCCESS) {
        cerr << "Could not get effective image resolution!" << endl;
        return UEYE_FAILED;
    }

    if(use_aoi) {
        if (aoi_img_width <= ueye_sInfo.nMaxWidth && aoi_img_left == -1) {
            cerr << "Left not specified, assuming center position" << endl;
            aoi_img_left = (ueye_sInfo.nMaxWidth - aoi_img_width) / 2;
        }
        if (aoi_img_height <= ueye_sInfo.nMaxHeight && aoi_img_top == -1) {
            cerr << "Top not specified, assuming center position" << endl;
            aoi_img_top = (ueye_sInfo.nMaxHeight - aoi_img_height) / 2;
        }

        IS_RECT rectAOI;
        rectAOI.s32X = aoi_img_left;
        rectAOI.s32Y = aoi_img_top;
        rectAOI.s32Width = aoi_img_width;
        rectAOI.s32Height = aoi_img_height;
        if (is_AOI(this->cameraHandle, IS_AOI_IMAGE_SET_AOI, (void *)&rectAOI,
                   sizeof(rectAOI)) != IS_SUCCESS) {
            cerr << "Cannot set area of interest (AOI)\n";
            return UEYE_FAILED;
        }
        ueye_img_width = aoi_img_width;
        ueye_img_height = aoi_img_height;

        cout << "Area of interest (AOI):\n";
        cout << "   Left: " << aoi_img_left << "\n";
        cout << "   Top: " << aoi_img_top << "\n";
        cout << "   Width: " << aoi_img_width << "\n";
        cout << "   Height: " << aoi_img_height << "\n";

    }

    if (getEffectiveImageResolution(&ueye_img_width,
                                    &ueye_img_height) != UEYE_SUCCESS) {
        cerr << "Could not get effective image resolution!" << endl;
        return UEYE_FAILED;
    } else {
        cout << "EffectiveImageResolution: width = "
             << ueye_img_width
             << ", height = " << ueye_img_height << endl;
    }

    image.create(ueye_img_height, ueye_img_width, CV_8UC(ueye_nBitsPerPixel / 8));
    //image = Mat(Size(ueye_img_width, ueye_img_height), CV_8UC(ueye_nBitsPerPixel/8));

    return UEYE_SUCCESS;
}


int UEyeCam::init(HIDS ueyeCameraID) {

    if ((ueyeCameraID < 0) or (ueyeCameraID > 254)) {
        return UEYE_FAILED;
    }

    if (initCam(ueyeCameraID) != UEYE_SUCCESS) {
        return UEYE_FAILED;
    }

    // Preparation for OneShot mode
    // memory initialization
    is_AllocImageMem(this->cameraHandle, ueye_img_width, ueye_img_height,
                     ueye_nBitsPerPixel, &ueye_pcImageMemory, &ueye_lMemoryId);
    // set memory active
    is_SetImageMem(this->cameraHandle, ueye_pcImageMemory, ueye_lMemoryId);
    is_SetDisplayMode(this->cameraHandle, IS_SET_DM_DIB);

    return UEYE_SUCCESS;
}

int UEyeCam::initContinous(HIDS ueyeCameraID) {

    if ((ueyeCameraID < 0) or (ueyeCameraID > 254)) {
        return UEYE_FAILED;
    }

    if (initCam(ueyeCameraID) != UEYE_SUCCESS) {
        return UEYE_FAILED;
    }

    // Preparation for continuous mode
    int nBufferSize = 3;
    ueye_lMemoryIds.clear();

    for (int i = 0; i < nBufferSize; i++) {
        /// grabber memory - buffer   ID.
        int lMemoryId;

        /// grabber memory - pointer to buffer.
        char* pcImageMemory;

        // memory initialization
        if (is_AllocImageMem(this->cameraHandle, ueye_img_width,
                             ueye_img_height, ueye_nBitsPerPixel,
                             &pcImageMemory, &lMemoryId) != IS_SUCCESS) {
            cerr << "Error while allocating image memory in libueyecam!" << endl;
            return UEYE_FAILED;
        }

        // Assign initialized memory to the queue (create a ring buffer)
        if (is_AddToSequence(this->cameraHandle, pcImageMemory,
                             lMemoryId) != IS_SUCCESS) {
            cerr << "AddToSequence went wrong!" << endl;
            return UEYE_FAILED;
        }

        cout << pcImageMemory << endl;
        ueye_lMemoryIds[pcImageMemory] = lMemoryId;
    }

    // Start the queue, unsuitable if images aren't processed fast enough!
    //is_InitImageQueue (this->cameraHandle, 0);

    is_SetDisplayMode(this->cameraHandle, IS_SET_DM_DIB);

    // Start continuous image capturing
    if (is_CaptureVideo(this->cameraHandle, IS_WAIT) != IS_SUCCESS) {
        cerr << "Could not start image captureing!" << endl;
        return UEYE_FAILED;
    }

    return UEYE_SUCCESS;
}

int UEyeCam::getImg(Mat &img, bool copy) {

    if (is_FreezeVideo(this->cameraHandle, IS_DONT_WAIT) != IS_SUCCESS) {
        cerr << "Error while capturing image!" << endl;
        return UEYE_FAILED;
    }

    image.data = (uchar*)ueye_pcImageMemory;

    // Get additional information about the image
    if (is_GetImageInfo(this->cameraHandle, ueye_lMemoryId, &ueye_imageInfo,
                        sizeof(ueye_imageInfo)) != IS_SUCCESS)
        cout << "UEyeCam::getImgContinous() - Could not get image info!" << endl;

    if (copy) {
        image.copyTo(img);  // return a copy of the image
    } else {
        // return the pointer to the image memory of the uEye camera
        img = image;
    }

    return UEYE_SUCCESS;
}

int UEyeCam::getImgContinous(Mat &img, bool copy) {
    /// Necessary if no copy is requested to know
    /// whether a image is stored
    static bool isLockedImg = false;

    /// Memory address of the last complete ring buffer of the camera
    static char *pBufferLast = NULL;

    int nNum = 0;
    char *pBuffer = NULL;

    // Free the ring buffer if still locked
    if (isLockedImg) {
        if (is_UnlockSeqBuf(this->cameraHandle,
                            IS_IGNORE_PARAMETER, pBufferLast) != IS_SUCCESS) {
            cerr << "Error unlocking image buffer!" << endl;
            return UEYE_FAILED;
        }
        isLockedImg = false;
    }

    // Test if new pointer to image memory was returned,
    // otherwise there is new image available
    char *pLastBufferLast = pBufferLast;

    // pBufferLast is most interesting here because it points to the
    // last image that was fully stored in memory
    if (is_GetActSeqBuf(this->cameraHandle, &nNum,
                        &pBuffer, &pBufferLast) != IS_SUCCESS) {
        cerr << "Could not get image buffer!" << endl;
        return UEYE_FAILED;
    }

    if (!pBufferLast) {
        cout << "Image buffer was empty!" << endl;
        return UEYE_SUCCESS;
    }

    // No new image
    if (pLastBufferLast == pBufferLast) {
        return UEYE_NO_NEW_IMG;
    }

    // Lock the image for further processing
    if (is_LockSeqBuf(this->cameraHandle, IS_IGNORE_PARAMETER,
                      pBufferLast) != IS_SUCCESS) {
        cerr << "Error locking image buffer!" << endl;
        return UEYE_FAILED;
    }
    isLockedImg = true;
    image.data = (uchar*)pBufferLast;

    // Buffer ID where currently is written to
    //cout << "test:" <<  nNum << endl;
    // Buffer ID, of the last write operation
    //cout << "test2:" <<  ueye_lMemoryIds[pBufferLast] << endl;

    // Return the image as copy or reference to the image buffer of the camera
    if (copy) {
        // Image can be freed after the copy.
        image.copyTo(img);
    } else {
        // Image is still locked, until this function gets called again
        img = image;
    }

    // Get more information about the image
    if (is_GetImageInfo(this->cameraHandle, ueye_lMemoryIds[pBufferLast],
                        &ueye_imageInfo, sizeof(ueye_imageInfo)) != IS_SUCCESS) {
//    unsigned long long u64TimestampDevice;
//    u64TimestampDevice = ueye_imageInfo.u64TimestampDevice;
        cout << "UEyeCam::getImgContinous() - Could not get image info!" << endl;
    }

    //cout << "Buffer: " << ueye_lMemoryIds[pBufferLast] << " Millisec: "
    //     << (int)ueye_imageInfo.TimestampSystem.wMilliseconds << endl;

    // Free the image if it was copied
    if (copy && isLockedImg) {
        if (is_UnlockSeqBuf(this->cameraHandle, IS_IGNORE_PARAMETER,
                            pBufferLast) != IS_SUCCESS) {
            cerr << "Error unlocking image buffer!" << endl;
            return UEYE_FAILED;
        }
        isLockedImg = false;
    }

//  INT nMemID = 0;
//  char *pBuffer = NULL;

    // based on a queue, unsuitable if images aren't processed fast enough!
//  if (is_WaitForNextImage(this->cameraHandle, 500, &pBuffer,
//                          &nMemID) != IS_SUCCESS) {
//    //is_SaveImageMem (m_hCam, "image.bmp", pBuffer, nMemID);
//    //is_UnlockSeqBuf (m_hCam, nMemID, pBuffer);
//    cerr << "Error while capturing image!" << endl;
//    return -1;
//  }
//  image.data = (uchar*)pBuffer;
//  image.copyTo(img);
//  is_UnlockSeqBuf(this->cameraHandle, nMemID, pBuffer);

    return UEYE_SUCCESS;
}

unsigned long long UEyeCam::getImgTimestamp() {
    //unsigned long long u64TimestampDevice;

    time_t rawtime;
    time(&rawtime);
    struct tm * timeinfo = localtime(&rawtime);
    // Year starts at 1900
    timeinfo->tm_year = ueye_imageInfo.TimestampSystem.wYear - 1900;
    // Month gets counted from 0 to 11
    timeinfo->tm_mon = ueye_imageInfo.TimestampSystem.wMonth - 1;
    timeinfo->tm_mday = ueye_imageInfo.TimestampSystem.wDay;
    timeinfo->tm_hour = ueye_imageInfo.TimestampSystem.wHour;
    timeinfo->tm_min = ueye_imageInfo.TimestampSystem.wMinute;
    timeinfo->tm_sec = ueye_imageInfo.TimestampSystem.wSecond;
    unsigned long long test = (unsigned long long)mktime(timeinfo) * 1000
                              + ueye_imageInfo.TimestampSystem.wMilliseconds;

    return test;
}

string UEyeCam::getTimeAsString() {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    timespec time1;
    clock_gettime(CLOCK_REALTIME, &time1);

    strftime(buffer, 80, "%y%m%d-%H%M%S", timeinfo);
    sprintf(buffer, "%s.%04d", buffer, (int)(time1.tv_nsec / 1000000));
    return string(buffer);
}

void UEyeCam::saveImg(std::string path, std::string extension) {
    std::string outputFile = path + getTimeAsString() + extension;
    imwrite(outputFile, image);
    cout << "Image saved under: " << outputFile.c_str() << "." << endl;
}

int UEyeCam::setExposureTime(double exposureTime, double *currentExposureTime) {

    // Set new exposure time
    if (is_Exposure(this->cameraHandle, IS_EXPOSURE_CMD_SET_EXPOSURE,
                    (void*)&exposureTime, sizeof(exposureTime)) != IS_SUCCESS)
        return UEYE_FAILED;

    // Ask for set exposure time:
    // (may be different from exposure time which was set before)
    if (is_Exposure(this->cameraHandle, IS_EXPOSURE_CMD_GET_EXPOSURE,
                    currentExposureTime,
                    sizeof(*currentExposureTime)) != IS_SUCCESS)
        return UEYE_FAILED;

    return UEYE_SUCCESS;
}

int UEyeCam::setAutoExposure(bool state) {

    double dEnable = state;
    if(is_SetAutoParameter (this->cameraHandle, IS_SET_ENABLE_AUTO_SHUTTER,
                            &dEnable, 0) != IS_SUCCESS)
        return UEYE_FAILED;

    return UEYE_SUCCESS;
}

int UEyeCam::getMaxExposure(double* maxExposure) {

    if(is_SetAutoParameter (this->cameraHandle, IS_GET_AUTO_SHUTTER_MAX,
                            maxExposure, 0) != IS_SUCCESS)
        return UEYE_FAILED;

    return UEYE_SUCCESS;
}

int UEyeCam::setFrameRate(double frameRate, double *currentFrameRate) {
    if (is_SetFrameRate(this->cameraHandle, frameRate,
                        currentFrameRate) != IS_SUCCESS) {
        return UEYE_FAILED;
    } else {
        return UEYE_SUCCESS;
    }
}

int UEyeCam::setHardwareGain(int nMaster, int nRed, int nGreen, int nBlue) {

    // Set new hardware gain
    if (is_SetHardwareGain(this->cameraHandle,
                           nMaster, nRed, nGreen, nBlue) != IS_SUCCESS)
        return UEYE_FAILED;

    return UEYE_SUCCESS;
}

int UEyeCam::setAutoGain(bool state) {

    double dEnable = state;
    if(is_SetAutoParameter (this->cameraHandle, IS_SET_ENABLE_AUTO_GAIN,
                            &dEnable, 0) != IS_SUCCESS)
        return UEYE_FAILED;

    return UEYE_SUCCESS;
}

int UEyeCam::isCamFree(int id) {

    printf("Searching Id: %d\n", id);
    // At least one camera has to e available
    INT nNumCam;
    if (is_GetNumberOfCameras(&nNumCam) == IS_SUCCESS) {
        if (nNumCam >= 1) {
            // Create a list with a suitable size
            UEYE_CAMERA_LIST* pucl;
            pucl = (UEYE_CAMERA_LIST*) new char [sizeof(DWORD)
                                                 + nNumCam
                                                 * sizeof(UEYE_CAMERA_INFO)];
            pucl->dwCount = nNumCam;
            // Read camera information
            if (is_GetCameraList(pucl) == IS_SUCCESS) {
                int iCamera;
                for (iCamera = 0; iCamera < (int)pucl->dwCount; iCamera++) {
                    // Print camera information
                    printf("Camera %i Id: %d inUSE: %d\n", iCamera,
                           pucl->uci[iCamera].dwCameraID,
                           pucl->uci[iCamera].dwInUse);
                    if (id == pucl->uci[iCamera].dwCameraID &&
                            pucl->uci[iCamera].dwInUse == 0) {
                        printf("Camera found and not in use\n");
                        return 1;
                    }
                }
            }
        }
    }

    printf("No Camera with this Id available\n");
    return 0;
}
