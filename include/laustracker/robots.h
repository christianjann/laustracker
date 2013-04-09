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

#ifndef HEADER_LAUSTRACKER_ROBOTS_H_INCLUDED
#define HEADER_LAUSTRACKER_ROBOTS_H_INCLUDED

/**
 * @file robots.h
 * @brief Contains the class RobotTracker, that can be used to locate robots.
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include <laustracker/map.h>

using namespace std;
using namespace cv;

/// Possible colors of a LED
typedef enum ColorEnum { c_red, c_blue, c_green, c_yellow, c_white } ColorType;

/// Contains information about a LED
typedef struct led_t
{
    /// LED position in pixels
    Point2f center;

    /// Radius/size of the LED
    float radius;

    /// Color of the LED
    ColorType color;
} led_t;

/// Stores information about a robot
typedef struct robot_t
{
    /// The blue LED at left front position.
    led_t led_blue;

    /// The red LED at right front position.
    led_t led_red;

    /// The center position of the robot in pixels
    Point center;

    /// The radius of a circle around the robot
    float radius;

    /// The index of the nearest field to the robot
    Point field_idx;

    /// Relative distance of the robot to the nearest field center point
    /// measured in field widths
    Point2f relative_pos;

    /// Absolute position of the robot in pixels, origin of ordinates is the
    /// left top corner of the labyrinth
    Point2f absolut_pos_px;

    /// Absolute position of the robot in cm, origin of ordinates is the
    /// left top corner of the labyrinth
    Point2f absolut_pos_cm;

    /// Orientation angle of the robot in degree
    float angle;

    /// Orientation angle of the robot, discretised in 4 directions
    /// (p_top, p_bottom, p_left, p_right)
    PosType direction;

    /// Name of the robot
    string name;

    /// Whether a third white LED was found
    bool third_led_found;

    /// The third white LED to distinguish different robots
    led_t led_white;

    /// Ratio of the distance from the blue LED to the white LED and the
    /// distance from the red LED to the white LED
    float ratio;

    /// Distance from the blue LED to the white LED
    float dist_blue_white;

    /// Distance from the red LED to the white LED
    float dist_red_white;

    /// Position f the robot within the last frame
    Point center_last_frame;

    /// Number of the frame where the robot was last found
    long long last_seen;

    /// To store temporary information about a robot, e.g. within loops
    int tmp_value;

} robot_t;

/**
 * @brief A class to locate robots
 *
 */
class RobotTracker
{
private:

    /// Color thresholds
    Scalar redmin1, redmax1, redmin2, redmax2, greenmin, greenmax,
           yellowmin, yellowmax, bluemin, bluemax, whitemin, whitemax;

    /// Show corresponding debug information
    int debuglevel;

    /// Frame counter
    long long frameNum;

    /// Field width in pixels
    int field_width_px;

    /// Labyrinth position (used to calculate robot positions)
    vector <Point> rob_labyrinth_pos;

    /// Clean image of the labyrinth at normal exposure time
    Mat labyrinth_image;

    /// We have a clean image of the labyrinth at normal exposure time
    bool have_labyrinth_image;

    /// Stores the center points of each field
    Point center_points[8][8];

    /// Is the labyrinth position already known
    bool labyrinth_pos_ready;

    /// Calculate center points for each field and store pixel positions
    void update_center_points();

    vector<String> robot_names;
    vector<float> robot_ratios;
    float max_ratio_diff;

    /// Skip not really necessary operations to get a higher FPS
    bool fastmode;

public:

    /// A list of all robots
    list< robot_t> robots;

    /**
     * \brief Constructor
     *
     * Initialization.
     *
     */
    RobotTracker();

    /**
     * \brief Deconstructor
     *
     * Free memory.
     *
     */
    ~RobotTracker();

    /**
     * \brief Load a robot configuration file.
     *
     * \param filename Relative path
     *
     */
    bool load_robots_conf(string filename);


    /**
     * \brief Search the image for robots and store robot positions.
     * \return true, if nothing went wrong
     */
    bool processNewFrame(Mat image);

    /// Return a list of all robots
    void getListOfRobots();

    /**
     * \brief Draw robots into a image
     * \param image Image to draw the robots into.
     */
    void draw_robots(Mat& image);

    /**
     * \brief Set the debug level.
     * \param level New value for the debug level
     */
    void setDebug(int level);

    /**
     * \brief Returns a list of all robots as string
     * \return String with robots
     */
    string toString();

    /**
     * \brief Load robots from string
     * \param robots_str String that contains robots
     */
    bool fromString(string robots_str);

    /**
     * \brief Set the labyrinth position.
     * \param labyrinth_pos Labyrinth position
     */
    void setLabyrinthPos(vector <Point> labyrinth_pos);

    /**
     * \brief Set the labyrinth position.
     * \param labyrinth_pos Labyrinth position
     * \param image A image of the labyrinth at normal exposure time.
     */
    void setLabyrinthPos(vector <Point> labyrinth_pos, Mat image);

    bool setRobotNames(vector<String> names, vector<float> ratios, float max_diff);

    /**
    * \brief Find the corresponding robot name from a list of robots and ratios
    * \param ratio Ratio of the distance from the blue LED to the white LED and the
    *        distance from the red LED to the white LED
    * \param debuglevel Show corresponding debug information
    * \return Name of the robot
    */
    String ratio2name(float ratio, int debuglevel);

    /**
     * \brief Set the fast mode.
     * \param state If true: skip not really necessary operations to get
     *        higher FPS
     */
    void setFastMode(bool state);
};

/**
 * \brief Find the corresponding robot name from a list of robots and distances
 * \param dist_blue_white Distance from the blue LED to the white LED
 * \param dist_red_white Distance from the red LED to the white LED
 * \return Name of the robot
 */
String dist2name(float dist_blue_white, float dist_red_white);

#endif
