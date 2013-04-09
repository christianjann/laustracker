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

#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ueyecam.h>
#include <laustracker/conf.h>
#include <laustracker/util.h>

#include <ltcamnode/setExposureTime.h>
#include <ltcamnode/setFrameRate.h>
#include <ltcamnode/setAutoGain.h>
#include <ltcamnode/setAutoExposure.h>
#include <ltcamnode/setHardwareGain.h>
#include <ltcamnode/getMaxExposureTime.h>

//http://www.boost.org/doc/libs/1_48_0/doc/html/program_options/tutorial.html
namespace po = boost::program_options;

int debuglevel = 0;
int maxfps = 20;

class UEyeCameraNode
{

private:

    UEyeCam myCamera;
    HIDS ueyeCameraID;
    std::string cameraName;

    cv::Mat capturedImage;
    cv_bridge::CvImage cvBridgeImage;

    ros::NodeHandle nodeHandle;
    ros::Rate *runLoopFrequency;
    sensor_msgs::Image rosImage;
    image_transport::Publisher imagePublisher;


public:

    UEyeCameraNode(ros::NodeHandle nodeHandle, int ueyeCameraID)
    {
        this->ueyeCameraID = ueyeCameraID;

    }

    bool setExposureTime(ltcamnode::setExposureTimeRequest &request,
                         ltcamnode::setExposureTimeResponse &response)
    {

        double currentExposureTime;

        if (this->myCamera.setExposureTime(request.exposureTime,
                                           &currentExposureTime) != UEYE_SUCCESS)
        {
            ROS_ERROR("Failed changing exposure time to %f ms",
                      request.exposureTime);
            response.currentExposureTime = currentExposureTime;
            return false;
        }
        else
        {
            ROS_INFO("Successfully changed exposure time to %f ms",
                     currentExposureTime);
            response.currentExposureTime = currentExposureTime;
            return true;
        }
    }

    bool setFrameRate(ltcamnode::setFrameRateRequest &request,
                      ltcamnode::setFrameRateResponse &response)
    {

        double currentFrameRate;

        if (this->myCamera.setFrameRate(request.framerate,
                                        &currentFrameRate) != UEYE_SUCCESS)
        {
            ROS_ERROR("Failed changing framerate to %f fps", request.framerate);
            response.ret = false;
            return false;
        }
        else
        {
            ROS_INFO("Successfully changed framerate to %lf fps", currentFrameRate);
            // Set the new loop frequency which specifies how long the main loop in
            // run() will sleep to avoid busy waiting.
            delete this->runLoopFrequency;
            this->runLoopFrequency = new ros::Rate(int(currentFrameRate));
            response.ret = true;
            return true;
        }
    }

    bool setAutoGain(ltcamnode::setAutoGainRequest &request,
                     ltcamnode::setAutoGainResponse &response)
    {

        if (this->myCamera.setAutoGain(request.state) != UEYE_SUCCESS)
        {
            ROS_ERROR("Failed changing autoGain to %i", request.state);
            response.ret = false;
            return false;
        }
        else
        {
            ROS_INFO("Successfully changed autoGain to %i", request.state);
            response.ret = true;
            return true;
        }
    }

    bool setAutoExposure(ltcamnode::setAutoExposureRequest &request,
                         ltcamnode::setAutoExposureResponse &response)
    {

        if (this->myCamera.setAutoExposure(request.state) != UEYE_SUCCESS)
        {
            ROS_ERROR("Failed changing autoExposure to %i", request.state);
            response.ret = false;
            return false;
        }
        else
        {
            ROS_INFO("Successfully changed autoExposure to %i", request.state);
            response.ret = true;
            return true;
        }
    }

    bool setHardwareGain(ltcamnode::setHardwareGainRequest &request,
                         ltcamnode::setHardwareGainResponse &response)
    {
        ;

        if (this->myCamera.setHardwareGain(request.nMaster, request.nRed,
                                           request.nGreen, request.nBlue) != UEYE_SUCCESS)
        {
            ROS_ERROR("Failed to change hardware gain (master: %i)", request.nMaster);
            response.ret = false;
            return false;
        }
        else
        {
            ROS_INFO("Successfully changed hardware gain (master: %i)", request.nMaster);
            response.ret = true;
            return true;
        }
    }

    bool getMaxExposureTime(ltcamnode::getMaxExposureTimeRequest &request,
                            ltcamnode::getMaxExposureTimeResponse &response)
    {

        double maxExposureTime;

        if (this->myCamera.getMaxExposure(&maxExposureTime) != UEYE_SUCCESS)
        {
            ROS_ERROR("Failed to get max exposure time");
            response.maxExposureTime = -1;
            return false;
        }
        else
        {
            ROS_INFO("Max exposure time: %f ms", maxExposureTime);
            response.maxExposureTime = maxExposureTime;
            return true;
        }
    }


    bool initializeCamera()
    {
        this->myCamera.setConfigurationFile(
            get_selfdir(debuglevel > 2) +
            "../../../../share/laustracker/UI124xLE-C_conf_aoi.ini");

        if (this->myCamera.initContinous(this->ueyeCameraID) != UEYE_SUCCESS)
        {
            return false;
        }
        else
        {
            /*
             * A ros::Rate object allows you to specify a frequency that you would
             * like to loop at. It will keep track of how long it has been since
             * the last call to Rate::sleep(), and sleep for the correct amount of time.
             * In this case we tell it we want to run at maxfps (20Hz).
             */
            this->runLoopFrequency = new ros::Rate(maxfps);
            return true;
        }
    }

    int run()
    {

        image_transport::ImageTransport cameraImageTransport =
            image_transport::ImageTransport(nodeHandle);

        ros::ServiceServer setExposureTimeService =
            nodeHandle.advertiseService("setExposureTime",
                                        &UEyeCameraNode::setExposureTime, this);
        ros::ServiceServer setFrameRateService =
            nodeHandle.advertiseService("setFrameRate",
                                        &UEyeCameraNode::setFrameRate, this);

        ros::ServiceServer setAutoExposure =
            nodeHandle.advertiseService("setAutoExposure",
                                        &UEyeCameraNode::setAutoExposure, this);

        ros::ServiceServer setAutoGain =
            nodeHandle.advertiseService("setAutoGain",
                                        &UEyeCameraNode::setAutoGain, this);

        ros::ServiceServer setHardwareGain =
            nodeHandle.advertiseService("setHardwareGain",
                                        &UEyeCameraNode::setHardwareGain, this);

        ros::ServiceServer getMaxExposureTime =
            nodeHandle.advertiseService("getMaxExposureTime",
                                        &UEyeCameraNode::getMaxExposureTime, this);

        imagePublisher = cameraImageTransport.advertise(
                             ros::names::remap("ueye_camera"), 1, false);

        int error_counter = 0;

        // framerate
        FpS fps;

        while (ros::ok())
        {
            if (imagePublisher.getNumSubscribers() > 0)
            {
                if (myCamera.getImgContinous(capturedImage, true) != UEYE_SUCCESS)
                {

                    error_counter++;
                    // don't spam us with too mush error messages
                    if (error_counter > 100)
                    {
                        ROS_ERROR("UEyeCam::getImgContinous() returned error!");
                        error_counter = 0;
                    }
                }
                else
                {
                    error_counter = 0;
                    cvBridgeImage.image = capturedImage;
                    cvBridgeImage.encoding = "bgr8";
                    cvBridgeImage.toImageMsg(rosImage);
                    imagePublisher.publish(rosImage);

                    // print current framerate
                    fps.measure();
                    fps.print();
                }
            }
            ros::spinOnce();
            this->runLoopFrequency->sleep();
        }

        ros::shutdown();
        return 0;

    }

};

int main(int argc, char **argv)
{

    int ueyeCameraID = 1;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "Show Help Message")
    ("fps", po::value<int>(&maxfps), "Set a upper FPS limit")
    ("id", po::value<int>(&ueyeCameraID), "Specify a camera id");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }

    cout << "Max FPS: " << maxfps << "\n";
    cout << "Camera ID: " << ueyeCameraID << "\n";

    ros::init(argc, argv, "uEyeCamera");
    ros::NodeHandle nodeHandle;

    UEyeCameraNode uEyeCameraNode(nodeHandle, ueyeCameraID);
    if (uEyeCameraNode.initializeCamera() != true)
    {
        ROS_ERROR("Could not initialize uEye camera.");
        ROS_INFO("Exiting.");
        ros::shutdown();
        exit(1);
    }
    uEyeCameraNode.run();
}
