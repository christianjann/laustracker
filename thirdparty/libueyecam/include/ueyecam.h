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

#ifndef UEYECAM_H_
#define UEYECAM_H_

// uEye.h needs a #define __LINUX__
// otherwise  it will try to compile for Windows! BUG????
#define __LINUX__ 1

#include <opencv2/opencv.hpp>
#include <map>
#include <ueye.h>
#include <string>

#define UEYE_NO_NEW_IMG 1
#define UEYE_SUCCESS	 0
#define UEYE_FAILED	-1


/**
* Get list of UEye cameras which are connected to the system.
*
* @return UEYE_CAMERA_LIST*
*/
UEYE_CAMERA_LIST* getUEyeCameraList();

/**
 * Simple Class for working with the uEye Cameras.
 */
class UEyeCam {

private:
    /// Grabber memory - buffer ID.
    int ueye_lMemoryId;

    /// Grabber memory - buffer IDs used by initContinuous() and is_addToSequence().
    std::map<char*,int> ueye_lMemoryIds;

    /// Grabber memory - pointer to buffer.
    char* ueye_pcImageMemory;

    /// Filename for loading camera configuration.
    std::string ueye_configIni;

    /// Camera identifier.
    HIDS ueyeCameraID;

    /// Camera handle.
    HIDS cameraHandle;

    /// Sensor info of the camera.
    SENSORINFO ueye_sInfo;

    cv::Mat image;

    /// Effective image width.
    int ueye_img_width;

    /// Effective image height.
    int ueye_img_height;

    int ueye_nBitsPerPixel;
    int ueye_nColorMode;

    /// Additional information of image capture, like a timestamp.
    UEYEIMAGEINFO ueye_imageInfo;

    /// Use area of interest (AOI).
    bool  use_aoi;

    /// Width of the area of interest (AOI)
    int aoi_img_width;

    /// Height of the area of interest (AOI)
    int aoi_img_height;

    /// X position of the area of interest (AOI)
    int aoi_img_left;

    /// Y position of the area of interest (AOI)
    int aoi_img_top;

    /**
     * Initialization.
     * Search for the first camera that is available.
     * Use the settings "parameter set" that are stored in that camera.
     * Search for the corresponding *.inf file in the current directory if
     * parameter set 1 does not exist else if that was not successful too
     * use default settings.
     *
     * @return UEYE_SUCCESS, if successful.
     */
    int initCam(HIDS ueyeCamera);

    /**
     * Helper function for saveImg(), to create a time stamp
     * in the form %y%m%d-%H:%M:%S for file names.
     */
    std::string getTimeAsString();

    /**
     * Load the camera configuration file specified in ueye_configIni.
     */
    bool loadConfiguration();

public:

    /**
     * Constructor
     *
     * Initialize variables
     */
    UEyeCam();

    /**
     * Deconstructor
     *
     * Free memory
     */
    ~UEyeCam();

    /**
     * Sets the file name to load a specific camera configuration file.
     * Possible configuration files are *.ini files that were created with
     * the example application QuEyeSdiDemo that ships with ueye_api or
     * internally stored configurations on the camera that get specified
     * with "cam/set1" or "cam/set2".
     *
     * @param configFile Filename of a configuration file or internal
     *                   stored parameter sets, look above.
     */
    void setConfigurationFile(std::string configFile);

    /**
     * Initialization for the OneShot mode.
     * For further details have a look at the initCam() function.
     *
     * @return UEYE_SUCCESS, if successful.
     */
    int init(HIDS ueyeCamera);

    /**
     * Initialization for the continuous capture mode.
     * For further details have a look at the initCam() function.
     *
     * @return UEYE_SUCCESS, if successful.
     */
    int initContinous(HIDS ueyeCamera);

    /**
     * Return current resolution of captured images (area of interest).
     *
     * @param ueyeCamera Camera handle.
     * @param width Pointer to an integer to store the image width.
     * @param height Pointer to an integer to store the image height.
     *
     * @return UEYE-SUCCESS on success and UEYE_FAILED if something goes wrong.
     */
    int getEffectiveImageResolution(int *width, int *height);

    /**
     * Set area of interest (AOI)
     *
     * Do this before init() or initContinous().
     *
     * @param width Width of the area of interest (AOI)
     * @param height Height of the area of interest (AOI)
     * @param left X position of the area of interest (AOI), -1 for image centre
     * @param top Y position of the area of interest (AOI), -1 for image centre
     *
     * @return UEYE-SUCCESS on success and UEYE_FAILED if something goes wrong.
     */
    int setAOI(int width, int height, int left = -1, int top = -1);

    /**
     * Capture a image from the camera.
     * Get a new image from the camera and store it in a OpenCV matrix.
     * OneShot mode.
     *
     * @param img Reference of a OpenCV matrix.
     * @param copy Specifies if the image gets copied into the matrix or
     *             if the memory address of the uEye image buffer is used.
     *
     * @return UEYE_SUCCESS, if successful, UEYE_FAILED else.
     */
    int getImg(cv::Mat& img,bool copy = 0);

    /**
     * Capture a image from the camera.
     * Get a new image from the camera and store it in a OpenCV matrix.
     * Continuous Capture mode.
     *
     * @param img Reference of a OpenCV matrix.
     * @param copy Specifies if the image gets copied into the matrix or
     *             if the memory address of the uEye image buffer is used.
     *
     * @return UEYE_SUCCESS, if successful, UEYE_FAILED else.
     */
    int getImgContinous(cv::Mat& img, bool copy = 0);

    /**
     * Return the timestamp for the last captured image.
     * Attention! The meaning of the timestamps is different whether a
     * GigE camera or a USB camera is used (have a look at the uEye manual:
     * is_GetImageInfo()).
     * Concerning GigE cameras the timestamp contains either the trigger time,
     * if used in OneShot mode or the time for begin of the image capturing
     * in continuous mode.
     * For USB cameras: it will return the time when the image was transferred
     * completely.
     */
    unsigned long long getImgTimestamp();

    /**
     * Stores the last captured image.
     * Attention! If the image capture function was used with copy=1 then
     * it is not guaranteed that the stored image is really the captured because
     * the image memory is not locked.
     */
    void saveImg(std::string path, std::string extension = ".png");

    /**
     * Sets the exposure time of the camera.
     *
     * @param exposureTime the exposure time to set
     * @param *currentExposureTime pointer pointing to the current exposure time
     *
     * @return UEYE_SUCCES, if succeeded, UEYE_FAILED if not
     */
    int setExposureTime(double exposureTime, double *currentExposureTime);

    /**
     * Enables/disables the auto exposure shutter function.
     *
     * @param state true to enable, false to disable
     * @return UEYE_SUCCES, if succeeded, UEYE_FAILED if not
     */
    int setAutoExposure(bool state);

    /**
     * Get maximum exposure time
     *
     * @param maxExposure Pointer to save the maximum exposure time
     * @return UEYE_SUCCES, if succeeded, UEYE_FAILED if not
     */
    int getMaxExposure(double* maxExposure);

    /**
     * Sets the frame rate of the camera.
     *
     * @param frameRate the frame rate to set
     * @param *currentFrameRate pointer pointing to the current frame rate
     *
     * @return UEYE_SUCCES, if succeeded, UEYE_FAILED if not
     */
    int setFrameRate(double frameRate, double *currentFrameRate);

    /**
     * Sets the hardware gain of the camera.
     *
     * @param nMaster Sets the overall gain factor (0...100).
     * @param nRed Sets the red channel gain factor (0...100).
     * @param nGreen Sets the green channel gain factor (0...100).
     * @param nBlue Sets the blue channel gain factor (0...100).
     *
     * @return UEYE_SUCCES, if succeeded, UEYE_FAILED if not
     */
    int setHardwareGain(int nMaster, int nRed = 12, int nGreen = 0, int nBlue = 18);

    /**
     * Enables/disables the auto gain function.
     *
     * @param state true to enable, false to disable
     * @return UEYE_SUCCES, if succeeded, UEYE_FAILED if not
     */
    int setAutoGain(bool state);

    int isCamFree(int id);

};

#endif /* UEYECAM_H_ */
