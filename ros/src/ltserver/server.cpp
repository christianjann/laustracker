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

#include <sstream>
#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <laustracker/laustracker.h>
#include <laustracker/conf.h>
#include <ueyecam.h>

#include <ltcamnode/setExposureTime.h>
#include <ltcamnode/setFrameRate.h>
#include <ltcamnode/setAutoGain.h>
#include <ltcamnode/setAutoExposure.h>
#include <ltcamnode/setHardwareGain.h>
#include <ltcamnode/getMaxExposureTime.h>

#include <ltserver/Robots.h>
#include <ltserver/Map.h>

//http://www.boost.org/doc/libs/1_48_0/doc/html/program_options/tutorial.html
namespace po = boost::program_options;
namespace enc = sensor_msgs::image_encodings;

bool map_ready = false, robots_ready = false;
LausTracker lt;

int debuglevel = 0;
long int frame_counter = 0;
int maxfps = 30;

template <typename PubType> void send_map(const PubType& pub)
{
    if (map_ready)
    {
        std_msgs::String msg;
        msg.data = lt.map.toString();
        if (debuglevel > 0) ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
    }
}

template <typename PubType> void send_map_msg(const PubType& pub)
{
    if (map_ready)
    {
        ltserver::Map msg;
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                msg.map.at(i * 8 + j).buoy_top = lt.map.fields[j][i].buoy_top;
                msg.map.at(i * 8 + j).buoy_bottom = lt.map.fields[j][i].buoy_bottom;
                msg.map.at(i * 8 + j).buoy_left = lt.map.fields[j][i].buoy_left;
                msg.map.at(i * 8 + j).buoy_right = lt.map.fields[j][i].buoy_right;
                msg.map.at(i * 8 + j).wall_top = lt.map.fields[j][i].wall_top;
                msg.map.at(i * 8 + j).wall_bottom = lt.map.fields[j][i].wall_bottom;
                msg.map.at(i * 8 + j).wall_left = lt.map.fields[j][i].wall_left;
                msg.map.at(i * 8 + j).wall_right = lt.map.fields[j][i].wall_right;
            }
        }
        pub.publish(msg);
    }
}

template <typename PubType> void send_robots(const PubType& pub)
{
    if (robots_ready)
    {
        std_msgs::String msg;
        msg.data = lt.robots.toString();
        if (debuglevel > 0) ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
    }
}

template <typename PubType> void send_robots_msg(const PubType& pub)
{
    if (robots_ready)
    {
        ltserver::Robots msg;
        for (list<robot_t>::iterator robot = lt.robots.robots.begin();
                robot != lt.robots.robots.end(); ++robot)
        {
            ltserver::Robot rob;
            rob.name = (*robot).name;
            rob.field_idx.x = (*robot).field_idx.x;
            rob.field_idx.y = (*robot).field_idx.y;
            rob.direction = (*robot).direction;
            rob.angle = (*robot).angle;
            rob.relative_pos.x = (*robot).relative_pos.x;
            rob.relative_pos.y = (*robot).relative_pos.y;
            rob.third_led_found = (*robot).third_led_found;
            rob.ratio = (*robot).ratio;
            rob.dist_blue_white = (*robot).dist_blue_white;
            rob.dist_red_white = (*robot).dist_red_white;
            rob.absolut_pos_cm.x = (*robot).absolut_pos_cm.x;
            rob.absolut_pos_cm.y = (*robot).absolut_pos_cm.y;

            msg.robots.push_back(rob);
        }
        pub.publish(msg);
    }
}

/*
 * Maybe use "latching" instead of clientConnect/clientDisconnect.
 * When a connection is latched, the last message published is saved
 * and automatically sent to any future subscribers that connect.
 * http://www.ros.org/wiki/roscpp/Overview/Publishers%20and%20Subscribers
 */
void mapClientConnect(const ros::SingleSubscriberPublisher& pub)
{
    string topic = pub.getTopic();
    static uint32_t mc_count = 0;
    std::stringstream ss;
    ss << "Map client connect #" << mc_count++
       << " " << topic;
    ROS_INFO("%s", ss.str().c_str());
    if (topic == "/map_publisher")
        send_map(pub);
    else //if(topic == "/map_msg_publisher")
        send_map_msg(pub);
}

void mapClientDisconnect(const ros::SingleSubscriberPublisher& pub)
{
    string topic = pub.getTopic();
    std::stringstream ss;
    ss << "Map client disconnect " << topic;
    ROS_INFO("%s", ss.str().c_str());
}

void robotsClientConnect(const ros::SingleSubscriberPublisher& pub)
{
    static uint32_t rc_count = 0;
    string topic = pub.getTopic();
    std::stringstream ss;
    ss << "Robots client connect #" << rc_count++
       << " " << topic;
    ROS_INFO("%s", ss.str().c_str());

    // cout << "typename: " << typeid(pub).name() <<"\n";
    if (topic == "/robots_publisher")
        send_robots(pub);
    else //if(topic == "/robots_msg_publisher")
        send_robots_msg(pub);
}

void robotsClientDisconnect(const ros::SingleSubscriberPublisher& pub)
{
    string topic = pub.getTopic();
    std::stringstream ss;
    ss << "Robots client disconnect " << topic;
    ROS_INFO("%s", ss.str().c_str());
}

void send_labyrinth_img(const Mat& image, vector <Point> rect,
                        image_transport::Publisher imagepub)
{
    if (imagepub.getNumSubscribers() > 0)
    {
        Mat cropped;
        Rect roi = boundingRect(rect);
        cropped = image(roi).clone();
        resize(cropped, cropped, Size(300, 300));
        flip(cropped, cropped, 0);
        cv_bridge::CvImage cvBridgeImage;
        sensor_msgs::Image rosImage;
        cvBridgeImage.image = cropped;
        cvBridgeImage.encoding = "bgr8";
        cvBridgeImage.toImageMsg(rosImage);
        imagepub.publish(rosImage);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, bool& new_frame, Mat& image)
{
    if (!(new_frame)) // check that old image has been processed?
    {
        frame_counter++;
        if (debuglevel > 2) ROS_INFO("I got a new Image: %lu\n", frame_counter);


        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
            image = cv_ptr->image.clone();
            new_frame = true;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

}

int main(int argc, char **argv)
{
    bool debug = false, undistort_image = true, use_walls_image = false,
         manual = false, use_robots_image = false, loacte_robots = false,
         use_camernode = false, wrap_image = true, use_mask = false,
         reddots = false, automatic = false, headless = false, load = false,
         use_robots_vid = false, fastmode = false;
    string robots_filename, walls_filename, camera_node, robots_conf_filename,
           labyrinth_conf_filename, robots_vid_filename;

    double normal_exp = UEYE_NORMAL_EXPOSURE_TIME; // normal exposure time in [ms]
    double dark_exp = UEYE_DARK_EXPOSURE_TIME;
    double current_exp = normal_exp; // current exposure time
    double new_exp = normal_exp; // helper variable
    double exposure_max = UEYE_EXPOSURE_MAX;
    int current_hw_gainmaster = UEYE_HW_GAIN_MASTER;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "Show Help Message")
    ("debug", "Show Debug Images")
    ("no-undistort", "Whether the image should not be undistorted")
    ("mask", "Use a mask image to dedect walls")
    ("manual", "Ask the user to select the labyrinth edges")
    ("reddots", "Use red dots to find the labyrinth edges")
    ("walls-img", po::value<string>(&walls_filename),
     "Use a image file and not the camera")
    ("robots-img", po::value<string>(&robots_filename),
     "Search for robots in an local image")
    ("robots-vid", po::value<string>(&robots_vid_filename),
     "Use a video file for endless robot tracking")
    ("camera-node", po::value<string>(&camera_node),
     "Search for robots in an local image")
    ("robots-conf", po::value<string>(&robots_conf_filename),
     "Use a specific robots configuration file")
    ("labyrinth-conf", po::value<string>(&labyrinth_conf_filename),
     "Use a specific labyrinth configuration file")
    ("debuglevel", po::value<int>(&debuglevel),
     "Set the debuglevel for Laustracker library functions")
    ("auto", "Locate labyrinth, walls, robots and adjust exposure time")
    ("headless", "Headless mode, don't display images")
    ("no-wrap", "Do not wrap the image, should be faster")
    ("fast", "Fast mode, the result may be a bit inaccurate but the FPS is higher")
    ("load", "Load labyrinth positition and walls from ~./laustracker/*.txt")
    ("fps", po::value<int>(&maxfps), "Set a upper FPS limit");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }

    if (vm.count("debug")) debug = true;
    if (vm.count("no-undistort")) undistort_image = false;
    if (vm.count("mask")) use_mask = true;
    if (vm.count("no-wrap")) wrap_image = false;
    if (vm.count("manual")) manual = true;
    if (vm.count("auto")) automatic = true;
    if (vm.count("headless")) headless = true;
    if (vm.count("reddots")) reddots = true;
    if (vm.count("load"))
    {
        load = true;
        manual = false;
        reddots = false;
    }
    if (vm.count("walls-img"))
    {
        cout << "Using image file: " << walls_filename << ".\n";
        use_walls_image = true;
        if (walls_filename.empty())
        {
            cout << "Missing walls-img filename.\n";
            return 1;
        }
    }
    if (vm.count("robots-img"))
    {
        cout << "Using image file: " << robots_filename << ".\n";
        use_robots_image = true;
        if (robots_filename.empty())
        {
            cout << "Missing robots-img filename.\n";
            return 1;
        }
    }
    if (vm.count("robots-vid"))
    {
        cout << "Using video file: " << robots_vid_filename << ".\n";
        use_robots_vid = true;
        if (robots_vid_filename.empty())
        {
            cout << "Missing robots-vid filename.\n";
            return 1;
        }
    }
    if (vm.count("camera-node"))
    {
        cout << "Using camera node: " << camera_node << ".\n";
        use_camernode = true;
        if (camera_node.empty())
        {
            cout << "Missing camera-node name.\n";
            return 1;
        }
    }

    lt.set_undistort(undistort_image);
    lt.set_wrap(wrap_image);
    lt.Undistort.set_intrinsics_xml("../../../../share/laustracker/intrinsics.xml");
    if (use_mask)
        lt.use_mask_to_locate_walls("../../../../share/laustracker/mask.png");

    if (vm.count("robots-conf"))
        lt.robots.load_robots_conf(robots_conf_filename);
    else
        lt.robots.load_robots_conf("../../../../share/laustracker/robots.conf");

    if (vm.count("labyrinth-conf"))
        lt.labyrinth_conf_filename = labyrinth_conf_filename;
    else
        lt.labyrinth_conf_filename = "../../../../share/laustracker/labyrinth.conf";

    if (vm.count("debug"))
    {
        lt.setDebug(4);
    }

    if (vm.count("debuglevel"))
    {
        cout << "debuglevel: " << debuglevel << ".\n";
        lt.setDebug(debuglevel);
    }
    if (vm.count("no-wrap")) lt.set_wrap(false);
    if (vm.count("fast"))
    {
        lt.robots.setFastMode(true);
        fastmode = true;
    }

    Mat walls_img, robots_img, frame;

    ros::init(argc, argv, "laustracker_server");
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<std_msgs::String>("map_publisher",
                             5, mapClientConnect, mapClientDisconnect);
    ros::Publisher map_msg_pub = nh.advertise<ltserver::Map>("map_msg_publisher",
                                 5, mapClientConnect, mapClientDisconnect);
    ros::Publisher robots_pub = nh.advertise<std_msgs::String>("robots_publisher",
                                5, robotsClientConnect, robotsClientDisconnect);
    ros::Publisher robots_msg_pub = nh.advertise<ltserver::Robots>("robots_msg_publisher",
                                    5, robotsClientConnect, robotsClientDisconnect);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher imagepub = it.advertise("laustracker/image", 5);

    // framerate
    FpS fps;

    ros::Rate loop_rate(maxfps);

    if (use_walls_image || use_robots_image || use_robots_vid)
    {
        if (load)
        {
            lt.load_labyrinth_pos();
            lt.map.load();
            map_ready = true;
            lt.printMap();
        }
        else if (use_walls_image)
        {
            walls_img = imread(walls_filename);
            if (!headless) lt.imshow("cv_ltserver", walls_img);

            if (manual)
            {
                if (!lt.locate_labyrinth_manually(lt.improve_image(walls_img)))
                    return 1;
            }
            else
            {
                lt.locate_labyrinth_automatically(lt.improve_image(walls_img), reddots);
            }


            walls_img = lt.improve_image(walls_img);
            if (!headless) lt.imshow("cv_ltserver", walls_img);

            Mat tmp_img;
            drawLabyrinthRect(walls_img, tmp_img, lt.get_labyrinth_pos());
            if (!headless) lt.imshow("cv_ltserver", tmp_img);
            lt.locate_walls(walls_img);
            if (!headless) lt.imshow("cv_ltserver_walls", walls_img);

            lt.printMap();
            string map = lt.map.toString();
            cout << "Original Map:\n";
            cout << map << "\n";
            lt.map.fromString(map);
            map = lt.map.toString();
            cout << "Map loaded from String:\n";
            cout << map << "\n";
            lt.printMap();
            map_ready = true;
        }
        if (use_robots_image)
        {
            robots_img = imread(robots_filename);
            robots_img = lt.improve_image(robots_img);
            lt.update_robot_positions(robots_img);
            //Mat tmp_img = robots_img;
            lt.robots.draw_robots(robots_img);
            if (!headless) lt.imshow("cv_ltserver_robots", robots_img);

            string robots = lt.robots.toString();
            cout << "Original Robots:\n";
            cout << lt.robots.toString() << "\n";
            lt.robots.fromString(robots);
            lt.printMap();
            cout << "Robots loaded from String:\n";
            cout << lt.robots.toString() << "\n";
            robots_ready = true;
        }
        else if (use_robots_vid)
        {
            if (fastmode)
                lt.set_wrap(false);
            VideoCapture inputVideo(robots_vid_filename); // Open input
            if (!inputVideo.isOpened())
            {
                cout  << "Could not open the input video: "
                      << robots_vid_filename << "\n";
                return -1;
            }
            //Show the image captured in the window and repeat
            while (ros::ok() && (char)waitKey(5) != 27)
            {
                // read
                inputVideo >> frame;
                // check if at end
                if (frame.empty())
                {
                    // repeat
                    inputVideo.set(CV_CAP_PROP_POS_FRAMES, 0);
                }
                else
                {
                    frame = lt.improve_image(frame);

                    if (debuglevel > 0) cout << "locate robots\n";
                    lt.update_robot_positions(frame);
                    lt.robots.draw_robots(frame);
                    if (!headless) lt.imshow("cv_ltserver_robots", frame);
                    robots_ready = true;
                    send_robots(robots_pub);
                    send_robots_msg(robots_msg_pub);

                    if (lt.labyrinthFound())
                        send_labyrinth_img(frame, lt.get_labyrinth_pos(), imagepub);

                    // print current framerate
                    fps.measure();
                    fps.print();

                    ros::spinOnce();
                    loop_rate.sleep();
                }

            }
            return 0;
        }

        ROS_INFO("Ready to serve the map.");

        while (ros::ok() && (char)waitKey(5) != 27)
        {
            ros::spinOnce();
            loop_rate.sleep();
            //if (use_robots_image)
            //  lt.update_robot_positions(robots_img);
            send_map(map_pub);
            send_map_msg(map_msg_pub);
            send_robots(robots_pub);
            send_robots_msg(robots_msg_pub);
            if (lt.labyrinthFound())
                send_labyrinth_img(robots_img, lt.get_labyrinth_pos(), imagepub);
        }

    }
    else
    {
        UEyeCam ucam;
        bool new_frame = false;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber imagesub;
        // The arguments that bind takes are copied and held internally by the
        // returned function object. boost::ref and boost::cref can be used
        // to make the function object store a reference to an object,
        // rather than a copy
        imagesub = it.subscribe("ueye_camera", 2,
                                boost::bind(imageCallback, _1, boost::ref(new_frame),
                                            boost::ref(frame)));

        ltcamnode::setExposureTime setExp;
        ltcamnode::setAutoGain setAutoGain;
        ltcamnode::setAutoExposure setAutoExposure;
        ltcamnode::setHardwareGain setHardwareGain;
        ltcamnode::getMaxExposureTime getMaxExposureTime ;

        ros::ServiceClient camnode_client_exp =
            nh.serviceClient<ltcamnode::setExposureTime>("setExposureTime");
        ros::ServiceClient camnode_client_auto_gain =
            nh.serviceClient<ltcamnode::setAutoGain>("setAutoGain");
        ros::ServiceClient camnode_client_auto_exp =
            nh.serviceClient<ltcamnode::setAutoExposure>("setAutoExposure");
        ros::ServiceClient camnode_client_hardware_gain =
            nh.serviceClient<ltcamnode::setHardwareGain>("setHardwareGain");
        ros::ServiceClient camnode_client_max_exposure =
            nh.serviceClient<ltcamnode::getMaxExposureTime>("getMaxExposureTime");


        if (use_camernode)
        {
            camnode_client_max_exposure.call(getMaxExposureTime);
            exposure_max = getMaxExposureTime.response.maxExposureTime;
            if (exposure_max == -1)
            {
                cout << "Failed to get max exposure time\n";
                exposure_max = UEYE_EXPOSURE_MAX;
            }

            // Quick & dirty automatic mode
            if (automatic)
            {
                cout << "Automatic mode: camera node\n";

                setAutoExposure.request.state = true;
                camnode_client_auto_exp.call(setAutoExposure);

                //setAutoGain.request.state = true;
                //camnode_client_auto_gain.call(setAutoGain);
                setHardwareGain.request.nMaster = UEYE_HW_GAIN_MASTER;
                setHardwareGain.request.nRed = UEYE_HW_GAIN_RED;
                setHardwareGain.request.nGreen = UEYE_HW_GAIN_GREEN;
                setHardwareGain.request.nBlue = UEYE_HW_GAIN_BLUE;
                camnode_client_hardware_gain.call(setHardwareGain);

                // let the camera adjust the exposure time automatically
                for (int i = 0; i < 5; i++)
                {
                    new_frame = false;
                    while (new_frame == false)
                    {
                        cout << "Waiting for frame\n";
                        sleep(1);
                        ros::spinOnce();
                    }
                    if (!headless) lt.imshow("cv_ltserver_automatic", frame);
                    waitKey(1);
                }

                setExp.request.exposureTime = dark_exp;
                camnode_client_exp.call(setExp);
                current_exp = setExp.response.currentExposureTime;

                setHardwareGain.request.nMaster = UEYE_HW_GAIN_MASTER_DARK;
                setHardwareGain.request.nRed = UEYE_HW_GAIN_RED;
                setHardwareGain.request.nGreen = UEYE_HW_GAIN_GREEN;
                setHardwareGain.request.nBlue = UEYE_HW_GAIN_BLUE;
                camnode_client_hardware_gain.call(setHardwareGain);
            }
        }
        else
        {
            ucam.setConfigurationFile(get_selfdir(debuglevel > 2) +
                                      "../../../../share/laustracker/UI124xLE-C_conf_aoi.ini");
            ucam.initContinous((HIDS) 0);

            // ucam.setDebug(false);
            // ucam.setExposureTime(normal_exp, &current_exp);
            ucam.getMaxExposure(&exposure_max);

            ucam.setHardwareGain(UEYE_HW_GAIN_MASTER, UEYE_HW_GAIN_RED,
                                 UEYE_HW_GAIN_GREEN, UEYE_HW_GAIN_BLUE);

            // Quick & dirty automatic mode
            if (automatic)
            {
                cout << "Automatic mode: local camera\n";
                // let the camera adjust the exposure time automatically
                for (int i = 0; i < 20; i++)
                    ucam.getImgContinous(frame);
                while (ucam.getImgContinous(frame) != UEYE_SUCCESS)
                {
                    cout << "Failed to get image for automatic mode\n";
                    sleep(1);
                }
                new_frame = true;
                cout << "Got frame for automatic mode\n";
                ucam.setExposureTime(dark_exp, &current_exp);

                ucam.setHardwareGain(UEYE_HW_GAIN_MASTER_DARK, UEYE_HW_GAIN_RED,
                                     UEYE_HW_GAIN_GREEN, UEYE_HW_GAIN_BLUE);
            }
        }

        // Quick & dirty automatic mode
        if (automatic)
        {
            cout << "Automatic mode: main\n";

            if (new_frame)
            {
                frame = lt.improve_image(frame);

                if (!headless) lt.imshow("cv_ltserver_automatic", frame);

                if (load)
                {
                    lt.load_labyrinth_pos();
                    lt.map.load();
                }
                else
                {
                    lt.locate_labyrinth_automatically(frame, reddots);
                    reddots = false; // do not again search for the labyrinth
                    lt.locate_walls(frame);
                }

                lt.printMap();
                if (!headless) lt.imshow("cv_ltserver_walls_automatic", frame);
                map_ready = true;
                send_map(map_pub);
                send_map_msg(map_msg_pub);
                send_labyrinth_img(frame, lt.get_labyrinth_pos(), imagepub);

                new_frame = false;
                loacte_robots = true;
                if (fastmode)
                    lt.set_wrap(false);
            }
        }

        //vector<String> names = {"Bernd", "Paul", "Karli", "Heinz", "Manu"};
        //vector<float> ratios = {1.42, 1.25, 1.03, 0.92, 0.81};
        //lt.robots.setRobotNames(names, ratios, 0.4);

        cout << "\npress ESC to exit\n";
        cout << "press  m  to locate the labyrinth manually\n";
        cout << "press  a  to locate the labyrinth automatically\n";
        cout << "press  e  to locate the labyrinth via red dots\n";
        cout << "press  w  to locate the walls\n";
        cout << "press  b  to locate the walls using the black lines\n";
        cout << "press  +  to increase the exposure time\n";
        cout << "press  -  to decrease the exposure time\n";
        cout << "press  n  to set the normal exposure time\n";
        cout << "press  d  to set the dark exposure time\n";
        cout << "press  s  to save the settings (labyrinth positition, walls)\n";
        cout << "press  l  to load settings (labyrinth positition, walls)\n";
        cout << "press  r  to toggle locating robots\n";
        cout << "press  1  to toggle the auto exposure shutter function.\n";
        cout << "press  2  to toggle the auto gain control function\n";
        cout << "press  key up  to increase hardware gain\n";
        cout << "press  key down  to decrease hardware gain\n\n";

        ROS_INFO("Ready to serve the map.");


        /*
         * A ros::Rate object allows you to specify a frequency that you would
         * like to loop at. It will keep track of how long it has been since
         * the last call to Rate::sleep(), and sleep for the correct amount of time.
         * In this case we tell it we want to run at maxfps (30Hz).
         */
        ros::Rate loop_rate(maxfps);
        bool automatically = false, manual = false;

        char key;

        while (ros::ok() && ((key = waitKey(1)) != 27))
        {
            if (!use_camernode)
            {
                new_frame = (ucam.getImgContinous(frame) == UEYE_SUCCESS);
            }
            if (new_frame)
            {
                frame = lt.improve_image(frame);
                if (!headless) lt.imshow("cv_ltserver", frame);

                if (manual)
                {
                    lt.locate_labyrinth_manually(frame);
                    manual = false;
                    //lt.set_wrap(true);
                }
                else if (automatically)
                {
                    lt.locate_labyrinth_automatically(frame);
                    automatically = false;
                }
                else if (reddots)
                {
                    lt.locate_labyrinth_automatically(frame, true);
                    reddots = false;
                }

                if (loacte_robots)
                {
                    if (debuglevel > 0) cout << "locate robots\n";
                    lt.update_robot_positions(frame);
                    lt.robots.draw_robots(frame);
                    if (!headless) lt.imshow("cv_ltserver_robots", frame);
                    robots_ready = true;
                    send_robots(robots_pub);
                    send_robots_msg(robots_msg_pub);
                }

                if (lt.labyrinthFound())
                    send_labyrinth_img(frame, lt.get_labyrinth_pos(), imagepub);

                new_frame = false;

                // print current framerate
                fps.measure();
                fps.print();

            }


            switch (key)
            {
            case 'm':
                // we need a new clean frame
                manual = true;
                lt.clear_labyrinth_pos();
                break;
            case 'a':
                // we need a new clean frame
                automatically = true;
                lt.clear_labyrinth_pos();
                break;
            case 'e':
                // we need a new clean frame
                reddots = true;
                lt.clear_labyrinth_pos();
                break;
            case 'w':
                lt.locate_walls(frame);
                lt.printMap();
                laustracker_imshow_croped("cv_ltserver_walls",
                                          frame, lt.get_labyrinth_pos());
                map_ready = true;
                send_map(map_pub);
                send_map_msg(map_msg_pub);
                send_labyrinth_img(frame, lt.get_labyrinth_pos(), imagepub);
                break;
            case '+':
                cout << "increase exposure time\n";
                if ((new_exp = current_exp + 1) < exposure_max)
                {
                    if (use_camernode)
                    {
                        setExp.request.exposureTime = new_exp;
                        camnode_client_exp.call(setExp);
                        current_exp = setExp.response.currentExposureTime;
                    }
                    else
                    {
                        ucam.setExposureTime(new_exp, &current_exp);
                    }
                    cout << "Exposure Time: " << current_exp << "\n";
                }
                else
                {
                    cout << "Max Exposure Time reached: " << current_exp << "\n";
                }
                break;
            case '-':
                cout << "decrease exposure time\n";
                if ((new_exp = current_exp - 1) > UEYE_EXPOSURE_MIN)
                {
                    if (use_camernode)
                    {
                        setExp.request.exposureTime = new_exp;
                        camnode_client_exp.call(setExp);
                        current_exp = setExp.response.currentExposureTime;
                    }
                    else
                    {
                        ucam.setExposureTime(new_exp, &current_exp);
                    }
                    cout << "Exposure Time: " << current_exp << "\n";
                }
                else
                {
                    cout << "Min Exposure Time reached: " << current_exp << "\n";
                }
                break;
            case 'n':
                cout << "set normal exposure time\n";
                if (use_camernode)
                {
                    setExp.request.exposureTime = normal_exp;
                    camnode_client_exp.call(setExp);
                    current_exp = setExp.response.currentExposureTime;

                    setHardwareGain.request.nMaster = UEYE_HW_GAIN_MASTER;
                    setHardwareGain.request.nRed = UEYE_HW_GAIN_RED;
                    setHardwareGain.request.nGreen = UEYE_HW_GAIN_GREEN;
                    setHardwareGain.request.nBlue = UEYE_HW_GAIN_BLUE;
                    camnode_client_hardware_gain.call(setHardwareGain);
                }
                else
                {
                    ucam.setExposureTime(normal_exp, &current_exp);
                    ucam.setAutoGain(UEYE_HW_GAIN_MASTER);
                }
                break;
            case 'd':
                cout << "set dark exposure time\n";
                if (use_camernode)
                {
                    setExp.request.exposureTime = dark_exp;
                    camnode_client_exp.call(setExp);
                    current_exp = setExp.response.currentExposureTime;

                    setHardwareGain.request.nMaster = UEYE_HW_GAIN_MASTER_DARK;
                    setHardwareGain.request.nRed = UEYE_HW_GAIN_RED;
                    setHardwareGain.request.nGreen = UEYE_HW_GAIN_GREEN;
                    setHardwareGain.request.nBlue = UEYE_HW_GAIN_BLUE;
                    camnode_client_hardware_gain.call(setHardwareGain);
                }
                else
                {
                    ucam.setExposureTime(dark_exp, &current_exp);
                    ucam.setAutoGain(UEYE_HW_GAIN_MASTER_DARK);
                }
                break;
            case 's':
                cout << "save settings\n";
                lt.save_labyrinth_pos();
                lt.map.save();
                break;
            case 'l':
                cout << "load settings\n";
                lt.load_labyrinth_pos();
                lt.map.load();
                map_ready = true;
                lt.printMap();
                send_map(map_pub);
                send_map_msg(map_msg_pub);
                break;
            case 'r':
                loacte_robots = !loacte_robots;
                if (fastmode && loacte_robots)
                    lt.set_wrap(false);
                else if (!loacte_robots)
                    lt.set_wrap(wrap_image);
                destroyWindow("cv_ltserver_robots");
                break;
            case '1':
                static bool autoexposure = false;
                autoexposure = !autoexposure;
                if (use_camernode)
                {
                    setAutoExposure.request.state = autoexposure;
                    if (!camnode_client_auto_exp.call(setAutoExposure))
                        cout << "Err setting autoexposure\n";
                }
                else
                {
                    if (ucam.setAutoExposure(autoexposure) != UEYE_SUCCESS)
                        cout << "Err setting autoexposure\n";

                }
                cout << "autoexposure = " << autoexposure << "\n";
                break;
            case '2':
                static bool autogain = false;
                autogain = !autogain;
                if (use_camernode)
                {
                    setAutoGain.request.state = autogain;
                    if (!camnode_client_auto_gain.call(setAutoGain))
                        cout << "Err setting autogain\n";
                }
                else
                {
                    if (ucam.setAutoGain(autogain) != UEYE_SUCCESS)
                        cout << "Err setting autogain\n";

                }
                cout << "aoutogain = " << autogain << "\n";
                break;
            case 82: // key up
                cout << "increase hardware gain\n";
                if (++current_hw_gainmaster < UEYE_HW_GAIN_MAX)
                {
                    if (use_camernode)
                    {
                        setHardwareGain.request.nMaster = current_hw_gainmaster;
                        setHardwareGain.request.nRed = UEYE_HW_GAIN_RED;
                        setHardwareGain.request.nGreen = UEYE_HW_GAIN_GREEN;
                        setHardwareGain.request.nBlue = UEYE_HW_GAIN_BLUE;
                        camnode_client_hardware_gain.call(setHardwareGain);
                    }
                    else
                    {
                        ucam.setHardwareGain(current_hw_gainmaster, UEYE_HW_GAIN_RED,
                                             UEYE_HW_GAIN_GREEN, UEYE_HW_GAIN_BLUE);
                    }
                    cout << "Hardware Gain Master: "
                         << current_hw_gainmaster << "\n";
                }
                else
                {
                    cout << "Max Hardware Gain reached: " << current_hw_gainmaster << "\n";
                }
                break;
            case 84: // key down
                cout << "decrease hardware gain\n";
                if (--current_hw_gainmaster > UEYE_HW_GAIN_MIN)
                {
                    if (use_camernode)
                    {
                        setHardwareGain.request.nMaster = current_hw_gainmaster;
                        setHardwareGain.request.nRed = UEYE_HW_GAIN_RED;
                        setHardwareGain.request.nGreen = UEYE_HW_GAIN_GREEN;
                        setHardwareGain.request.nBlue = UEYE_HW_GAIN_BLUE;
                        camnode_client_hardware_gain.call(setHardwareGain);
                    }
                    else
                    {
                        ucam.setHardwareGain(current_hw_gainmaster, UEYE_HW_GAIN_RED,
                                             UEYE_HW_GAIN_GREEN, UEYE_HW_GAIN_BLUE);
                    }
                    cout << "Hardware Gain Master: "
                         << current_hw_gainmaster << "\n";
                }
                else
                {
                    cout << "Min Hardware Gain reached: " << current_hw_gainmaster << "\n";
                }
                break;
            default:
                if (key != -1)
                    cout << "Unknown key pressed: " << int(key) << "\n";
                break;
            }

            ros::spinOnce();
            loop_rate.sleep();
            // usleep(1000);

        }
    }

    ros::shutdown();
    return 0;
}
