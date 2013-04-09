/*
 * This file is part of Laustracker.
 *
 * Copyright (C) 2013 Christian Jann <christian.jann@ymail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// console application to save a camera stream
// http://docs.opencv.org/doc/tutorials/highgui/video-write/video-write.html
// http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html

// You may have to compile OpenCV yourself with ffmpeg support to write .avi files

#include <laustracker/laustracker.h>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <ueyecam.h>

using namespace std;
using namespace cv;

static void help()
{
    cout
            << "This program shows how to write video files.\n"
            << "Usage:\n"
            << "./laustracker-video-write outputvideoName\n";
}

int main(int argc, char* argv[])
{

    if (argc != 2)
    {
        cout << "\nERROR: Wrong number of parameters\n";
        help();
        return -1;
    }

    Mat frame;
//     VideoCapture cap(0); // open the default camera
//     if (!cap.isOpened()) // check if we succeeded
//         return -1;
//     cap >> frame;

    UEyeCam ucam;
    ucam.setConfigurationFile(get_selfdir(true) +
                              "../share/laustracker/UI124xLE-C_conf_aoi.ini");
    ucam.initContinous((HIDS) 0);

    double current_exp = UEYE_DARK_EXPOSURE_TIME;
    ucam.setExposureTime(UEYE_DARK_EXPOSURE_TIME, &current_exp);
    ucam.setHardwareGain(30, UEYE_HW_GAIN_RED,
                         UEYE_HW_GAIN_GREEN, UEYE_HW_GAIN_BLUE);

    // framerate
    FpS fps;

    while (ucam.getImgContinous(frame) != UEYE_SUCCESS)
    {
        sleep(1);
        cout << "could not get frame\n";
    }

    VideoWriter outputVideo;
    // Open the output
    outputVideo.open(argv[1], CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(frame.cols, frame.rows), true);

    if (!outputVideo.isOpened())
    {
        cout  << "Could not open the output video for write: " << argv[1] << "\n";
        return -1;
    }

    namedWindow("Captured Video", 1);
    while ((char)waitKey(1) != 27)
    {
        //cap >> frame; // get a new frame from camera
        if (ucam.getImgContinous(frame) == UEYE_SUCCESS)
        {

            //flip(frame, frame, 0); // flip image vertical
            imshow("Captured Video", frame);

            //outputVideo.write(res); //save or
            outputVideo << frame;

            // print current framerate
            fps.measure();
            fps.print();
        }
    }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
