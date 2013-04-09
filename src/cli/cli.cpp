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

#include <laustracker/laustracker.h>
#include <laustracker/conf.h>
#include <ueyecam.h>

#include <iostream>
#include <boost/program_options.hpp>

//http://www.boost.org/doc/libs/1_48_0/doc/html/program_options/tutorial.html
namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    bool debug = false, undistort_image = true, use_walls_image = false,
         manual = false, use_robots_image = false, loacte_robots = false,
         use_mask = false, reddots = false, use_black_lines = false;
    string robots_filename, walls_filename, robots_conf_filename,
           labyrinth_conf_filename;

    double normal_exp = UEYE_NORMAL_EXPOSURE_TIME; // normal exposure time in [ms]
    double dark_exp = UEYE_DARK_EXPOSURE_TIME;
    double current_exp = normal_exp; // current exposure time
    double new_exp = normal_exp; // helper variable
    double exposure_max = UEYE_EXPOSURE_MAX;
    int current_hw_gainmaster = UEYE_HW_GAIN_MASTER;

    int debuglevel = 0;

    // framerate
    FpS fps;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "Show Help Message")
    ("debug", "Show Debug Images")
    ("no-undistort", "Whether the image should not be undistorted")
    ("mask", "Use a mask image to dedect walls")
    ("black", "Use black lines to dedect walls")
    ("manual", "Ask the user to select the labyrinth edges")
    ("reddots", "Use red dots to find the labyrinth edges")
    ("walls-img", po::value<string>(&walls_filename),
     "Use a image file and not the camera")
    ("robots-img", po::value<string>(&robots_filename),
     "Search for robots in an local image")
    ("robots-conf", po::value<string>(&robots_conf_filename),
     "Use a specific robots configuration file")
    ("labyrinth-conf", po::value<string>(&labyrinth_conf_filename),
     "Use a specific labyrinth configuration file")
    ("debuglevel", po::value<int>(&debuglevel),
     "Set the debuglevel for Laustracker library functions")
    ("benchmark", "Use robots-img to measure the performance (max FPS)")
    ("no-wrap", "Do not wrap the image, should be faster")
    ("fast", "Fast mode, the result may be a bit inaccurate but the FPS is higher");

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
    if (vm.count("black")) use_black_lines = true;
    if (vm.count("manual")) manual = true;
    if (vm.count("reddots")) reddots = true;
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

    LausTracker lt;
    lt.set_undistort(undistort_image);

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
    if (vm.count("fast")) lt.robots.setFastMode(true);

    if (use_mask)
        lt.use_mask_to_locate_walls("../share/laustracker/mask.png");
    if (use_black_lines)
        lt.use_mask_to_locate_walls("../share/laustracker/mask_black_lines.png");

    if (vm.count("robots-conf"))
        lt.robots.load_robots_conf(robots_conf_filename);
    if (vm.count("labyrinth-conf"))
        lt.labyrinth_conf_filename = labyrinth_conf_filename;

    Mat walls_img, robots_img, frame;

    if (use_walls_image || use_robots_image)
    {
        if (use_walls_image)
        {
            walls_img = imread(walls_filename);
            lt.imshow("cv_laustracker_cli", walls_img);

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
            lt.imshow("cv_laustracker_cli", walls_img);

            Mat tmp_img;
            drawLabyrinthRect(walls_img, tmp_img, lt.get_labyrinth_pos());
            lt.imshow("cv_laustracker_cli", tmp_img);
            if (use_black_lines)
                lt.locate_black_lines(walls_img);
            else
                lt.locate_walls(walls_img);
            lt.imshow("cv_laustracker_cli_walls", walls_img);

            lt.printMap();
            string map = lt.map.toString();
            cout << "Original Map:\n";
            cout << map << "\n";
            lt.map.fromString(map);
            map = lt.map.toString();
            cout << "Map loaded from String:\n";
            cout << map << "\n";
            cout << "Save map to file\n";
            lt.map.save();
            cout << "Load map from file\n";
            lt.map.load();
            lt.printMap();

            cout << "Save labyrinth positition to file:\n";
            cout << lt.get_labyrinth_pos() << "\n";
            lt.save_labyrinth_pos();
            lt.clear_labyrinth_pos();
            cout << "Load labyrinth positition from file\n";
            lt.load_labyrinth_pos();
            cout << lt.get_labyrinth_pos() << "\n";
        }
        if (use_robots_image)
        {
            robots_img = imread(robots_filename);
            robots_img = lt.improve_image(robots_img);
            lt.update_robot_positions(robots_img);
            lt.robots.draw_robots(robots_img);
            lt.imshow("cv_laustracker_cli_robots", robots_img);

            string robots = lt.robots.toString();
            cout << "Original Robots:\n";
            cout << lt.robots.toString() << "\n";
            lt.robots.fromString(robots);
            lt.printMap();
            cout << "Robots loaded from String:\n";
            cout << lt.robots.toString() << "\n";

            if (vm.count("benchmark"))
            {
                Mat new_robots_img = imread(robots_filename);

                while (((char)waitKey(1)) != 27)
                {
                    Mat tmp_img = new_robots_img.clone();
                    tmp_img = lt.improve_image(tmp_img);
                    lt.update_robot_positions(tmp_img);
                    lt.robots.draw_robots(tmp_img);
                    lt.imshow("cv_laustracker_cli_robots", tmp_img);

                    // print current framerate
                    fps.measure();
                    fps.print();
                }
                exit(0);
            }
        }
        waitKey();
    }
    else
    {
        UEyeCam ucam;
        ucam.setConfigurationFile(get_selfdir(debuglevel > 2) +
                                  "../share/laustracker/UI124xLE-C_conf_aoi.ini");
        //ucam.setAOI(680, 680);
        //ucam.setAOI(680, 680, 290, 160);
        ucam.initContinous((HIDS) 0);
        //ucam.setDebug(false);
        //ucam.setExposureTime(normal_exp, &current_exp);
        //ucam.setHardwareGain(current_hw_gainmaster, UEYE_HW_GAIN_RED,
        //                     UEYE_HW_GAIN_GREEN, UEYE_HW_GAIN_BLUE);
        ucam.getMaxExposure(&exposure_max);

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

        bool automatically = false, manual = false, reddots = false;

        char key;

        while ((key = waitKey(1)) != 27) // wait one millisecond for key event
        {

            if (ucam.getImgContinous(frame) == UEYE_SUCCESS)
            {

                if (debug) lt.imshow("cv_before_improve", frame);

                frame = lt.improve_image(frame);
                lt.imshow("cv_laustracker_cli", frame);

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
                    if (debug) cout << "locate robots\n";
                    lt.update_robot_positions(frame);
                    lt.robots.draw_robots(frame);
                    lt.imshow("cv_laustracker_cli_robots", frame);
                }

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
                lt.imshow("cv_laustracker_cli_walls", frame);
                break;
            case 'b':
                lt.locate_black_lines(frame);
                lt.printMap();
                lt.imshow("cv_laustracker_cli_walls", frame);
                break;
            case '+':
                cout << "increase exposure time\n";
                if ((new_exp = current_exp + 1) < exposure_max)
                {
                    ucam.setExposureTime(new_exp, &current_exp);
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
                    ucam.setExposureTime(new_exp, &current_exp);
                    cout << "Exposure Time: " << current_exp << "\n";
                }
                else
                {
                    cout << "Min Exposure Time reached: " << current_exp << "\n";
                }
                break;
            case 'n':
                cout << "set normal exposure time\n";
                ucam.setExposureTime(normal_exp, &current_exp);
                break;
            case 'd':
                cout << "set dark exposure time\n";
                ucam.setExposureTime(dark_exp, &current_exp);
                break;
            case 's':
                cout << "save settings\n";
                //TODO see ltserver
                break;
            case 'l':
                cout << "load settings\n";
                //TODO see ltserver
                break;
            case 'r':
                loacte_robots = !loacte_robots;
                destroyWindow("cv_laustracker_cli_robots");
                break;
            case '1':
                static bool autoexposure = false;
                if (ucam.setAutoExposure(autoexposure = !autoexposure) == UEYE_SUCCESS)
                    cout << "autoexposure = " << autoexposure << "\n";
                break;
            case '2':
                static bool autogain = false;
                if (ucam.setAutoGain(autogain = !autogain) == UEYE_SUCCESS)
                    cout << "aoutogain = " << autogain << "\n";
                break;
            case 82: // key up
                cout << "increase hardware gain\n";
                if (++current_hw_gainmaster < UEYE_HW_GAIN_MAX)
                {
                    ucam.setHardwareGain(current_hw_gainmaster, UEYE_HW_GAIN_RED,
                                         UEYE_HW_GAIN_GREEN, UEYE_HW_GAIN_BLUE);
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
                    ucam.setHardwareGain(current_hw_gainmaster, UEYE_HW_GAIN_RED,
                                         UEYE_HW_GAIN_GREEN, UEYE_HW_GAIN_BLUE);
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
        }
    }
    return 0;
}
