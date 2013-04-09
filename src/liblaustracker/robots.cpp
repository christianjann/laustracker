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

#include <laustracker/robots.h>
#include <laustracker/image_manipulation.h>
#include <laustracker/util.h>
#include <laustracker/labyrinth.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <sstream>
#include <iomanip>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

RobotTracker::RobotTracker()
{
    debuglevel = 0;
    frameNum = -1;
    field_width_px = 70;
    labyrinth_pos_ready = false;
    load_robots_conf("../share/laustracker/robots.conf");
    fastmode = false;
    have_labyrinth_image = false;
}

RobotTracker::~RobotTracker()
{
}

bool RobotTracker::load_robots_conf(string filename)
{
    bool ret = true;
    boost::property_tree::ptree robot_conf;
    try
    {
        cout << "Trying to open " << filename << "\n";
        boost::property_tree::ini_parser::read_ini(
            get_selfdir(debuglevel > 2) + filename, robot_conf);
    }
    catch (...)
    {
        ret = false;
        cout << "Warning: could not open " << filename << "\n";
    }

    if (ret)
    {
        /*
         * HSV colors
         * Gimp:
         *       H: 0..360
         *       S: 0..100
         *       V: 0..100
         * OpenCV:
         *       H: 0..179
         *       S: 0..255
         *       V: 0..255
         * Open a image with Gimp, select the "Color Picker Tool",
         * click on the palette "Foreground Color" then you see
         * the color in "Change Foreground Color".
         */

        // http://stackoverflow.com/questions/12204522/efficiently-threshold-red-using-hsv-in-opencv
        redmin1 = get_gimp_color(robot_conf, "gimp_colors.hsv_redmin_1");
        redmax1 = get_gimp_color(robot_conf, "gimp_colors.hsv_redmax_1");
        redmin2 = get_gimp_color(robot_conf, "gimp_colors.hsv_redmin_2");
        redmax2 = get_gimp_color(robot_conf, "gimp_colors.hsv_redmax_2");

        greenmin = get_gimp_color(robot_conf, "gimp_colors.hsv_greenmin");
        greenmax = get_gimp_color(robot_conf, "gimp_colors.hsv_greenmax");

        yellowmin = get_gimp_color(robot_conf, "gimp_colors.hsv_yellowmin");
        yellowmax = get_gimp_color(robot_conf, "gimp_colors.hsv_yellowmax");

        bluemin = get_gimp_color(robot_conf, "gimp_colors.hsv_bluemin");
        bluemax = get_gimp_color(robot_conf, "gimp_colors.hsv_bluemax");

        whitemin = get_gimp_color(robot_conf, "gimp_colors.hsv_whitemin");
        whitemax = get_gimp_color(robot_conf, "gimp_colors.hsv_whitemax");

        robot_names = to_array<std::string>(robot_conf.get<std::string>("robots.names"));
        robot_ratios = to_array<float>(robot_conf.get<std::string>("robots.ratios"));
        max_ratio_diff = robot_conf.get<float>("robots.max_ratio_diff");

        cout << "robot_names = " << robot_names << "\n";
        cout << "robot_ratios = " << robot_ratios << "\n";
        cout << "max_ratio_diff = " << max_ratio_diff << "\n";
    }
    return ret;
}

bool RobotTracker::setRobotNames(vector<String> names, vector<float> ratios,
                                 float max_diff)
{
    if (robot_names.size() == robot_ratios.size())
    {
        robot_names = names;
        robot_ratios = ratios;
        max_ratio_diff = max_diff;
        for (int i = 0; i < robot_names.size(); i++)
        {
            cout << robot_names[i] << " ratio: " << robot_ratios[i]
                 << " max diff: " << max_ratio_diff << "\n";
        }
        return true;
    }
    else
        return false;
}
String RobotTracker::ratio2name(float ratio, int debuglevel = 0)
{
    static int counter = 0;
    bool found_name = false;
    String name = "";
    float max_diff = max_ratio_diff;

//     String names[] = {"Bernd", "Paul", "Karli", "Heinz", "Manu"};
//     float ratios[] = {1.22, 1.11, 0.98, 1.66, 0.85};

    // find the robot that fits best
    for (int i = 0; i < robot_names.size(); i++)
    {
        if (debuglevel > 2) cout << "comparing ratio " << i << ": "
                                     << robot_ratios[i] << " with " << ratio << "\n";
        if (abs(robot_ratios[i] - ratio) < max_diff)
        {
            max_diff = abs(robot_ratios[i] - ratio);
            found_name = true;
            name = robot_names[i];
            if (debuglevel > 2) cout << "name=names[" << i << "]: "
                                         << robot_names[i] << " new max_diff="
                                         << max_diff << " \n";
        }
    }

    if (!found_name)
    {
        stringstream tmp;
        tmp << "robot_" << counter;
        name = tmp.str();
        counter++;
    }
    return name;
}

String dist2name(float dist_blue_white, float dist_red_white)
{
    static int counter = 0;
    bool found_name = false;
    String name = "";
    float max_diff = 2;

    String names[] = {"Bernd", "Paul", "Karli", "Heinz", "Manu"};
    float dist_bw[] = {36, 32, 34, 17, 20};
    float dist_rw[] = {43, 36, 33, 29, 17};

    // find the robot that fits best
    for (int i = 0; i < 5; i++)
    {
        // Calculate a scale factor
        float factor = dist_bw[i] / dist_blue_white;
        cout << "factor: " << factor << "\n";
        // multiply the second distance with the scale factor
        float calculated_rw = factor * dist_red_white;
        // compare each distance with calculated value
        cout << "comparing dist_rw " << i << ": " << dist_rw[i]
             << " with calculated_rw: " << calculated_rw << "\n";
        if (abs(dist_rw[i] - calculated_rw) < max_diff)
        {
            max_diff = abs(dist_rw[i] - calculated_rw);
            found_name = true;
            name = names[i];
            cout << "name=names[" << i << "]: " << names[i]
                 << " new max_diff=" << max_diff << " \n";
        }
    }

    if (!found_name)
    {
        stringstream tmp;
        tmp << "robot_" << counter;
        name = tmp.str();
        counter++;
    }
    return name;
}

template <typename point> point real_pos_roi(const Point& origin,
        const point& p)
{
    return point(p.x + origin.x, p.y + origin.y);
}

bool RobotTracker::processNewFrame(Mat image)
{
    Mat img, blue, red, white, color_img;

    bool use_image_roi = labyrinth_pos_ready && fastmode;

    if (!labyrinth_pos_ready)
        cout << "RobotTracker::processNewFrame: warning labyrinth_pos unknown\n";

    if (use_image_roi)
    {
        // define image ROI
        img = image(Rect(rob_labyrinth_pos[0], rob_labyrinth_pos[2]));
        if (debuglevel > 3) laustracker_imshow("Robots: Region of Interest", img);
    }
    else
    {
        img = image;
    }

    if (debuglevel > 3) color_img = image.clone();

    Mat hsv_img, tmp_img;
    cvtColor(img, hsv_img, CV_BGR2HSV);

    /// blue
    inRange(hsv_img, bluemin, bluemax, blue);
    if (fastmode) edm(blue, blue, 0, 1, 0);
    else edm(blue, blue, 1, 1, 5);
    if (debuglevel > 3) laustracker_imshow("cv_robots_blue_thres", blue);

    /// red
    // http://stackoverflow.com/questions/12204522/efficiently-threshold-red-using-hsv-in-opencv
    inRange(hsv_img, redmin1, redmax1, tmp_img);
    inRange(hsv_img, redmin2, redmax2, red);
    bitwise_or(tmp_img, red, red);
    if (fastmode) edm(red, red, 0, 1, 0);
    else edm(red, red, 1, 1, 5);
    if (debuglevel > 3) laustracker_imshow("cv_robots_red_thres", red);

    /// white
    inRange(hsv_img, whitemin, whitemax, white);
    // sometimes red gets detected as white, so remove that
    bitwise_not(red, white, white);
    if (fastmode) edm(white, white, 0, 1, 0);
    else edm(white, white, 1, 1, 5);
    if (debuglevel > 3) laustracker_imshow("cv_robots_white_thres", white);

    vector<led_t> red_leds;
    vector<led_t> blue_leds;
    vector<led_t> white_leds;
    list<robot_t > tmp_robots;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Find red LEDS
    findContours(red, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    if (debuglevel > 2) cout << "Found " << contours.size() << " red contours\n";
    for (int n = 0; n < (int)contours.size(); n++)
    {
        led_t led;
        minEnclosingCircle(contours[n], led.center , led.radius);
        if (use_image_roi)
            led.center = real_pos_roi(rob_labyrinth_pos[0], led.center);
        led.color = c_red;
        red_leds.push_back(led);
        if (debuglevel > 3)
            circle(color_img, led.center, (int)led.radius * 2,
                   Scalar(0, 0, 255), 2, 8, 0);
    }

    /// Find blue LEDs
    findContours(blue, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    if (debuglevel > 2) cout << "Found " << contours.size() << " blue contours\n";
    for (int n = 0; n < (int)contours.size(); n++)
    {
        led_t led;
        minEnclosingCircle(contours[n], led.center , led.radius);
        if (use_image_roi)
            led.center = real_pos_roi(rob_labyrinth_pos[0], led.center);
        led.color = c_blue;
        blue_leds.push_back(led);
        if (debuglevel > 3)
            circle(color_img, led.center, (int)led.radius * 2,
                   Scalar(255, 0, 0), 2, 8, 0);
    }

    /// Find white LEDs
    findContours(white, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    if (debuglevel > 2) cout << "Found " << contours.size() << " white contours\n";
    for (int n = 0; n < (int)contours.size(); n++)
    {
        led_t led;
        minEnclosingCircle(contours[n], led.center , led.radius);
        if (use_image_roi)
            led.center = real_pos_roi(rob_labyrinth_pos[0], led.center);
        led.color = c_white;
        white_leds.push_back(led);
        if (debuglevel > 3)
            circle(color_img, led.center, (int)led.radius * 2,
                   Scalar(255, 255, 255), 2, 8, 0);
    }
    if (debuglevel > 3) laustracker_imshow("cv_robots_found_leds", color_img);
    //waitKey();

    int num = -1;

    if (debuglevel > 4)
    {
        if (red_leds.empty())
            laustracker_imshow("cv_robots_no_red_led", color_img);
        if (blue_leds.empty())
            laustracker_imshow("cv_robots_no_blue_led", color_img);
        if (white_leds.empty())
            laustracker_imshow("cv_robots_no_white_led", color_img);
    }

    if (!red_leds.empty() && !blue_leds.empty())
    {
        for (int i = 0; i < red_leds.size(); i++)
        {
            for (int j = 0; j < blue_leds.size(); j++)
            {
                int dist = DistancePointPoint(red_leds[i].center,
                                              blue_leds[j].center);

                if (debuglevel > 2) cout << "dist: " << dist
                                             << " < (field_width_px * 0.6): "
                                             << field_width_px * 0.6 << " ?\n";
                if (dist < field_width_px * 0.6)
                {
                    if (debuglevel > 2) cout << "OK: Distance between red LED["
                                                 << i << "] and blue LED[" << j
                                                 << "]: " << dist << " Pixel\n";
                    robot_t rob;
                    num++;
                    stringstream name;
                    name << "robot_" << num;
                    rob.name = name.str();
                    rob.tmp_value = 0;

                    rob.third_led_found = false;
                    rob.ratio = -1;

                    rob.led_blue = blue_leds[j];
                    rob.led_red = red_leds[i];

                    Point2f p1 = centerPointPoint(red_leds[i].center,
                                                  blue_leds[j].center);
                    // normal vector
                    int x = -(red_leds[i].center.y - blue_leds[j].center.y);
                    int y = (red_leds[i].center.x - blue_leds[j].center.x);

                    rob.center.x = p1.x + x * 0.5;
                    rob.center.y = p1.y + y * 0.5;

                    p1.x = p1.x - x * 0.5;
                    p1.y = p1.y - y * 0.5;

                    rob.radius = dist * 1;

                    if (debuglevel > 3)
                    {
                        circle(color_img, rob.center, rob.radius,
                               Scalar(0, 255, 0), 2, 8, 0);
                        line(color_img, rob.center, p1, Scalar(0, 255, 0), 2);
                    }

                    rob.field_idx = Point(0, 0);

                    if (debuglevel > 2) cout << "pc:" << rob.center << "\n";
                    if (debuglevel > 2) cout << "p1:" << p1 << "\n";

                    float dx, dy;
                    dy = rob.center.y - p1.y;
                    dx = rob.center.x - p1.x;
                    if (debuglevel > 2) cout << "dx:" << dx
                                                 << " dy:" << dy << "\n";
                    // Calculate the orientation angle
                    if (dx > 0 && dy >= 0)
                    {
                        rob.angle = 270 + degrees(atan(dy / dx));
                    }
                    else if (dx > 0 && dy < 0)
                    {
                        rob.angle = 270 + degrees(atan(dy / dx));
                    }
                    else if (dx < 0 && dy < 0)
                    {
                        rob.angle = 90 + degrees(atan(dy / dx));
                    }
                    else if (dx < 0 && dy >= 0)
                    {
                        rob.angle = 90 + degrees(atan(dy / dx));
                    }
                    else if (dx == 0)
                    {
                        if (dy > 0)
                            rob.angle = 0;
                        else
                            rob.angle = 180;
                    }
                    else
                    {
                        cout << "error: wrong values\n";
                    }

                    if (debuglevel > 2) cout << "rob.angle=" << rob.angle << "\n";

                    // Calculate the orientation angle of the robot,
                    // discretised in 4 directions
                    if (rob.angle >= 315 || rob.angle < 45)
                        rob.direction = P_TOP;
                    else if (rob.angle >= 45 && rob.angle < 135)
                        rob.direction = P_RIGHT;
                    else if (rob.angle >= 135 && rob.angle < 225)
                        rob.direction = P_BOTTOM;
                    else if (rob.angle >= 225 && rob.angle < 315)
                        rob.direction = P_LEFT;
                    else
                        rob.direction = P_OTHER;

                    if (labyrinth_pos_ready)
                    {
                        Point top_left = rob_labyrinth_pos[0];
                        Point top_right = rob_labyrinth_pos[1];

                        // Find the nearest field for the robot
                        float min_distance = top_right.x - top_left.x;
                        for (int y = 0; y < 8; y++)
                        {
                            for (int x = 0; x < 8; x++)
                            {
                                float distance =
                                    DistancePointPoint(rob.center,
                                                       center_points[x][y]);
                                if (distance < min_distance)
                                {
                                    min_distance = distance;
                                    rob.field_idx = Point(x, y);
                                }
                            }
                        }

                        if (debuglevel > 2) cout << "rob.field_idx=["
                                                     << rob.field_idx.x << ","
                                                     << rob.field_idx.y << "]\n";

                        rob.absolut_pos_px = rob.center - top_left;

                        float field_width_cm = 30.0;
                        Point2f pos;
                        pos.x = rob.absolut_pos_px.x / field_width_px * field_width_cm;
                        pos.y = rob.absolut_pos_px.y / field_width_px * field_width_cm;
                        rob.absolut_pos_cm = pos;

                        if (debuglevel > 2) cout << rob.name
                                                     << " rob.absolut_pos_cm="
                                                     << rob.absolut_pos_cm << "\n";

                        Point2f rel_pos = center_points[rob.field_idx.x]
                                          [rob.field_idx.y] - rob.center;
                        rel_pos.x = rel_pos.x / field_width_px;
                        rel_pos.y = rel_pos.y / field_width_px;
                        rob.relative_pos = rel_pos;

                        if (debuglevel > 2) cout << rob.name << " rob.relative_pos="
                                                     << rob.relative_pos << "\n";

                        if (debuglevel > 4)
                        {
                            circle(color_img,
                                   center_points[rob.field_idx.x][rob.field_idx.y],
                                   4, Scalar(0, 0, 255), CV_FILLED);
                            line(color_img, center_points[rob.field_idx.x]
                                 [rob.field_idx.y], rob.center, Scalar(0, 0, 255));

                            Point calc_r_pos = center_points[rob.field_idx.x][rob.field_idx.y] -
                                               Point(rob.relative_pos.x * field_width_px,
                                                     rob.relative_pos.y * field_width_px);
                            circle(color_img, calc_r_pos, 4,
                                   Scalar(255, 255, 255), CV_FILLED);
                        }
                    }

                    tmp_robots.push_back(rob);
                }
                else
                {
                    if (debuglevel > 2) cout << "NOT OK: Distance between red LED["
                                                 << i << "] and blue LED[" << j
                                                 << "]: " << dist << " Pixel\n";
                }
            }
        }
        if (debuglevel > 3) laustracker_imshow("cv_robots_found_leds", color_img);

    }
    else
    {
        if (debuglevel > 2) cout << "Not enough LEDs found\n";
    }

    // Assign white LEDs to the robots
    if (tmp_robots.size() > 0)
    {
        for (list<robot_t>::iterator robot = tmp_robots.begin();
                robot != tmp_robots.end(); ++robot)
        {
            if (debuglevel > 2) cout << "\nsearching white leds for "
                                         << (*robot).name << "\n";
            float max_dist = (*robot).radius; // * 1.2;
            if (debuglevel > 2) cout << "max distance: " << max_dist << "\n";
            for (int i = 0; i < white_leds.size(); i++)
            {
                // Exclude white LEDs from another robot and remove false positives
                float dist = DistancePointPoint((*robot).center, white_leds[i].center);
                if (debuglevel > 2) cout << "checking white led " << i << ": "
                                             << dist << "<" << max_dist << "\n";

                if (dist < max_dist)
                {
                    // The white LED should be at the back of the robot
                    float d_line_point = DistanceLinePoint(
                                             (*robot).led_blue.center,
                                             (*robot).led_red.center,
                                             white_leds[i].center, false);
                    if (debuglevel > 2) cout << "d_line_point: " << d_line_point << "\n";
                    bool behind = d_line_point > (*robot).radius / 4;

                    if (behind)
                    {
                        float dist_blue_white = DistancePointPoint((*robot).led_blue.center,
                                                white_leds[i].center);
                        float dist_red_white = DistancePointPoint((*robot).led_red.center,
                                               white_leds[i].center);

                        if (debuglevel > 2) cout << "dist_blue_white: "
                                                     << dist_blue_white << "\n";
                        if (debuglevel > 2) cout << "dist_red_white: "
                                                     << dist_red_white << "\n";

                        //float ang = angle((*robot).led_red.center,
                        //                  white_leds[i].center,
                        //                  (*robot).led_blue.center);
                        //cout << "angle: " << degrees(ang) << "\n";
                        float ratio = dist_red_white / dist_blue_white;
                        if (debuglevel > 2) cout << "ratio: " << ratio << "\n";
                        (*robot).third_led_found = true;
                        (*robot).ratio = ratio;
                        (*robot).dist_blue_white = dist_blue_white;
                        (*robot).dist_red_white = dist_red_white;
                        //(*robot).name = dist2name(dist_blue_white, dist_red_white);
                        (*robot).name = ratio2name(ratio);
                        (*robot).led_white = white_leds[i];
                        break;
                    }
                }
            }

        }
    }

    // remove robots without white LED
    for (auto robot = tmp_robots.begin(); robot != tmp_robots.end();)
    {
        if (!(*robot).third_led_found)
        {
            if (debuglevel > 2) cout << "Deleting robot: "
                                         << (*robot).name << "\n";
            robot = tmp_robots.erase(robot);
        }
        else
            ++robot;
    }

    // Remove other unwanted robots
    if (!tmp_robots.empty())
    {
        for (auto robota = tmp_robots.begin(); robota != tmp_robots.end(); ++robota)
        {
            for (auto robotb = robota; ++robotb != tmp_robots.end(); /**/)
            {
                if (debuglevel > 2) cout << "Comparing " << (*robota).name
                                             << " and " << (*robotb).name << "\n";

                // Remove robots that are to near at each other
                int dist = DistancePointPoint((*robota).center, (*robotb).center);
                if (debuglevel > 2) cout << "   Distance: " << dist << "\n";
                bool to_near = dist < ((*robota).radius + (*robotb).radius) * 0.8;
                if (debuglevel > 2)
                    if (to_near) cout << "   too near\n";

                if (to_near)
                {
                    // Keep the robot where the LEDs are best visible
                    if ((*robota).led_blue.radius +
                            (*robota).led_red.radius <
                            ((*robotb).led_blue.radius +
                             (*robotb).led_red.radius))
                        (*robota).tmp_value = 1;
                    else
                        (*robotb).tmp_value = 1;
                }

            }
        }
        for (auto robot = tmp_robots.begin(); robot != tmp_robots.end();)
        {
            if ((*robot).tmp_value == 1)
            {
                if (debuglevel > 2) cout << "Deleting robot: "
                                             << (*robot).name << "\n";
                robot = tmp_robots.erase(robot);
            }
            else
                ++robot;
        }

    }

    if (tmp_robots.size() > 0)
    {
        robots = tmp_robots;
        return true;
    }
    else
    {
        //only now, until we have a real robot tracking
        robots.clear();

        return false;
    }
}

void RobotTracker::draw_robots(Mat& image)
{
    for (list<robot_t>::iterator robot = robots.begin();
            robot != robots.end(); ++robot)
    {
        circle(image, (*robot).led_red.center, (int)((*robot).led_red.radius * 2),
               Scalar(0, 0, 255), 2, 8, 0);
        circle(image, (*robot).led_blue.center, (int)((*robot).led_blue.radius * 2),
               Scalar(255, 0, 0), 2, 8, 0);
        circle(image, (*robot).center, (*robot).radius, Scalar(0, 255, 0), 2, 8, 0);
        Point2f p1 = centerPointPoint((*robot).led_blue.center,
                                      (*robot).led_red.center);

        // Normalenvektor
        int x = -((*robot).led_red.center.y - (*robot).led_blue.center.y);
        int y = ((*robot).led_red.center.x - (*robot).led_blue.center.x);
        p1.x = p1.x - x * 0.5;
        p1.y = p1.y - y * 0.5;
        line(image, (*robot).center, p1, Scalar(0, 255, 0), 2);

        if ((*robot).third_led_found)
            circle(image, (*robot).led_white.center,
                   (int)((*robot).led_white.radius * 2),
                   Scalar(255, 255, 255), 2, 8, 0);
    }
}

void RobotTracker::setDebug(int level)
{
    debuglevel = level;
}

void RobotTracker::setFastMode(bool state)
{
    fastmode = state;
}

void RobotTracker::setLabyrinthPos(vector <Point> labyrinth_pos)
{
    rob_labyrinth_pos = labyrinth_pos;
    // We may need the inner border of the labyrinth to get correct robot positions
    //scale_rect(rob_labyrinth_pos , -10);
    field_width_px = (rob_labyrinth_pos[1].x -
                      rob_labyrinth_pos[0].x) / 8.0;
    if (debuglevel > 2) cout << "field_width_px: " << field_width_px << "\n";
    labyrinth_pos_ready = true;
    update_center_points();
}

void RobotTracker::setLabyrinthPos(vector <Point> labyrinth_pos, Mat image)
{
    setLabyrinthPos(labyrinth_pos);
    labyrinth_image = image.clone();
    have_labyrinth_image = true;
}

void RobotTracker::update_center_points()
{
    Point top_left = rob_labyrinth_pos[0];
    Point top_right = rob_labyrinth_pos[1];
    Point bottom_right = rob_labyrinth_pos[2];
    Point bottom_left = rob_labyrinth_pos[3];
    int fieldwidth_top = (top_right.x - top_left.x) / 8;
    int fieldwidth_bottom = (bottom_right.x - bottom_left.x) / 8;
    int fieldheight_right = (bottom_right.y - top_right.y) / 8;
    int fieldheight_left = (bottom_left.y - top_left.y) / 8;

    Mat tmp_image;
    if (debuglevel > 3 && have_labyrinth_image) tmp_image = labyrinth_image.clone();
    Point p1, p2, p3, p4;
    for (int y = 0; y < 8; y++)
    {
        for (int x = 0; x < 8; x++)
        {
            p1.x = top_left.x;
            p1.y = top_left.y + y * fieldheight_left + fieldheight_left / 2;
            p2.x = top_right.x;
            p2.y = top_right.y + y * fieldheight_right + fieldheight_right / 2;

            p3.x = top_left.x + x * fieldwidth_top + fieldwidth_top / 2;
            p3.y = top_left.y;
            p4.x = bottom_left.x + x * fieldwidth_bottom + fieldwidth_bottom / 2;
            p4.y = bottom_left.y;

            Point center;
            intersection(p1, p2, p3, p4, center);
            center_points[x][y] = center;
            if (debuglevel > 3 && have_labyrinth_image)
            {
                drawLabyrinthRect(tmp_image, tmp_image, rob_labyrinth_pos);
                circle(tmp_image, center, 5, Scalar(0, 0, 255), 2);
                line(tmp_image, p1, p2, Scalar(0, 255, 0), 2);
                line(tmp_image, p3, p4, Scalar(0, 255, 0), 2);
                laustracker_imshow_croped("cv_center_points", tmp_image, rob_labyrinth_pos);
            }
        }
    }

    if (debuglevel > 3 && have_labyrinth_image)
    {
        // test field_width_px
        Point a, b;
        a = Point(center_points[0][0].x - field_width_px / 2, center_points[0][0].y);
        b = Point(a.x + field_width_px, a.y);
        line(tmp_image, a, b, Scalar(255, 255, 255), 2);

        // draw origin of ordinates
        circle(tmp_image, top_left, 10, Scalar(0, 0, 255), CV_FILLED);

        laustracker_imshow_croped("cv_center_points", tmp_image, rob_labyrinth_pos);
    }
}

string RobotTracker::toString()
{
    stringstream buf;
    buf << "START_ROBOTS:\n";

    for (list<robot_t>::iterator robot = robots.begin(); robot != robots.end(); ++robot)
    {
        //buf << std::fixed << std::setprecision(3); // save 3 digits
        buf << (*robot).name << " ";
        buf << (*robot).field_idx.x << " " << (*robot).field_idx.y << " ";
        buf << (*robot).direction << " " << (*robot).angle << " ";
        buf << (*robot).relative_pos.x << " " << (*robot).relative_pos.y << " ";
        buf << (*robot).third_led_found << " " << (*robot).ratio << " ";
        buf << (*robot).dist_blue_white << " " << (*robot).dist_red_white << " ";
        buf << (*robot).absolut_pos_cm.x << " " << (*robot).absolut_pos_cm.y << " ";

        // Add new values here, also update Python script and RobotTracker::fromString
        // (but not necessary if you add new values at the end)

        buf << "\n";
    }

    buf << ":END_ROBOTS";
    return buf.str();
}

bool RobotTracker::fromString(string robot_str)
{
    robots.clear();

    string marker_begin = "START_ROBOTS:";
    string marker_end = ":END_ROBOTS";
    int begin = robot_str.find(marker_begin);
    int end = robot_str.find(marker_end);

    if ((begin > -1) && (end > -1))
    {
        istringstream stream(robot_str.substr(begin + marker_begin.length() + 1,
                                              end - begin - marker_begin.length() - 1));
        string line;
        while (getline(stream, line))
        {
            if (debuglevel > 2) cout << "line: " << line << "\n";
            robot_t rob;
            stringstream ss(line);

            // buf << (*robot).name<< " ";
            ss >> rob.name;

            // buf << (*robot).field_idx.x << " " << (*robot).field_idx.y << " ";
            int x, y; ss >> x; ss >> y; rob.field_idx = Point(x, y);

            // buf << (*robot).direction << " " << (*robot).angle << " ";
            int d; ss >> d; rob.direction = (PosType)d; ss >> rob.angle;

            // buf << (*robot).relative_pos.x << " " << (*robot).relative_pos.y << " ";
            float fx, fy; ss >> fx; ss >> fy; rob.relative_pos = Point2f(fx, fy);

            // buf << (*robot).third_led_found<< " " << (*robot).ratio << " ";
            ss >> rob.third_led_found; ss >> rob.ratio;

            // buf << (*robot).dist_blue_white << " " << (*robot).dist_red_white << " ";
            ss >> rob.dist_blue_white; ss >> rob.dist_red_white;

            // buf << (*robot).absolut_pos_cm.x << " " << (*robot).absolut_pos_cm.y << " ";
            ss >> fx; ss >> fy; rob.absolut_pos_cm = Point2f(fx, fy);

            // Add new values here

            robots.push_back(rob);
        }

    }
    else if (begin > -1)
    {
        cout << "robots: only found begin\n";
        return false;
    }
    else if (end > -1)
    {
        cout << "robots: only found end\n";
        return false;
    }
    else
    {
        cout << "no robots found\n";
        return false;
    }

}
