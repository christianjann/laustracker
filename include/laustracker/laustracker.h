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

#ifndef HEADER_LAUSTRACKER_LAUSTRACKER_H_INCLUDED
#define HEADER_LAUSTRACKER_LAUSTRACKER_H_INCLUDED
/**
 * @file laustracker.h
 * @brief This header file will include all public headers of liblaustracker.
 *
 *  So you only need to include a single file:
 *  @code #include <laustracker/laustracker.h> @endcode
 */

/** \mainpage Liblaustracker
 *
 * \image html screenshot_Laustracker3D.png
 * \section intro_sec Intro:
 *
 * For more information, please visit http://laustracker.jann.cc.
 *
 * \section install_sec Usage:
 * \subsection step1 1. Include the laustracker header file:
 *             @code #include <laustracker/laustracker.h> @endcode
 * \subsection step2 2. Link against liblaustracker, e.g.:
 *             @code -llaustracker @endcode
 *
 * \author Christian Jann
 * \version 1.0.0
 *
 * <!--\defgroup examples Example programs-->
 * <!--\defgroup lib Library functions-->
 */

#include <laustracker/version.h>
#include <laustracker/map.h>
#include <laustracker/util.h>
#include <laustracker/image_manipulation.h>
#include <laustracker/color_conversion.h>
#include <laustracker/labyrinth.h>
#include <laustracker/robots.h>

/**
 * @brief Class to manage all information about the labyrinth position and
 *        structure as well as the robot positions.
 */
class LausTracker
{

private:
    /// The labyrinth position
    vector <Point> labyrinth_pos;

    /**
     * Print debug messages and show debug images.
     *
     * debuglevel:
     *
     *   - = 0: No debug output
     *   - > 0: Enable some debug messages
     *   - > 1: Additional delay functions
     *   - > 2: Print debug messages
     *   - > 3: Show debug images
     *   - > 4: Additional debug output
     */
    int debuglevel;

    /// Was the labyrinth found?
    bool labyrinth_found;

    /// Whether a perspective transformation is needed
    bool wrap;

    /// Undistort the image
    bool undistort;

    /// Instance of PerspectiveWrapper
    PerspectiveWrapper Wrapper;

    bool use_mask;
    String mask_filename;

    /// File to save or load the labyrinth position
    string labyrinth_pos_path;

public:

    /// Instance of LabyrinthMap
    LabyrinthMap map;

    /// Instance of RobotTracker
    RobotTracker robots;

    /// Instance of ImageUndistort
    ImageUndistort Undistort;

    /// Path to a labyrinth configuration file.
    String labyrinth_conf_filename;

    /**
     * \brief Constructor
     *
     * Initialization
     *
     */
    LausTracker();

    /**
     * \brief Deconstructor
     *
     * Free memory.
     *
     */
    ~LausTracker();

    /**
     * \brief The user should manually click at every labyrinth corner.
     *
     * \param image Image of the labyrinth.
     *
     * \return true, if the user has selected four corners that form a quad.
     *
     */
    bool locate_labyrinth_manually(const Mat& image);

    /**
     * \brief Try to find the labyrinth automatically.
     *
     * \param image Image of the labyrinth.
     *
     * \param reddots Use red markers at the labyrinth corners to locate
     *        the labyrinth.
     *
     * \return true, if something was found the could be the labyrinth.
     *
     */
    bool locate_labyrinth_automatically(const Mat& image, bool reddots = false);

    /**
     * \brief Returns the labyrinth position.
     * \return  The labyrinth position.
     */
    vector <Point> get_labyrinth_pos();

    /**
     * \brief Save the labyrinth position to ~/.laustracker/labyrinth_pos.txt.
     *
     * \return true, if the labyrinth position could be saved.
     *
     */
    bool save_labyrinth_pos();

    /**
     * \brief Load the labyrinth position from ~/.laustracker/labyrinth_pos.txt.
     *
     * \return true, if the labyrinth position was loaded.
     *
     */
    bool load_labyrinth_pos();

    /**
     * \brief Clear the labyrinth position.
     */
    void clear_labyrinth_pos();

    /**
     * \brief Check the existence of walls.
     * \param image Image of the labyrinth to check for walls.
     */
    void locate_walls(Mat& image);

    /**
     * \brief Check the existence of walls by checking the visibility of black
     *        lines.
     * \param image Image of the labyrinth to check for walls.
     */
    void locate_black_lines(Mat& image);

    /**
     * \brief Update robot positions.
     * \param image Image of the labyrinth to check for robots.
     */
    void update_robot_positions(Mat image);

    /**
     * \brief Show debug information.
     * \param level Debuglevel
     */
    void setDebug(int level);

    // bool save_labyrinth_pos();
    // bool load_labyrinth_pos();

    /**
     * \brief Is the labyrinth position already available.
     * \return true, if the labyrinth position is known
     */
    bool labyrinthFound();

    /// Print the labyrinth map
    void printMap();

    /**
     * \brief Improve camera images (undistort, wrap)
     * \param image The source image
     * \return Improved image
     */
    Mat improve_image(const Mat& image);

    /**
     * \brief Display a image and automatically crop it if the
     *        labyrinth position is known-
     * \param windoname Window title
     * \param image Image to display
     */
    void imshow(const string& windoname, const Mat& image);

    /**
     * \brief Change the undistort setting
     * \param state New value
     */
    void set_undistort(bool state);

    /**
     * \brief The wrap setting, whether a perspective transformation should be done.
     * \param state New value
     */
    void set_wrap(bool state);

    /**
     * \brief Set a mask image to locate walls.
     * \param mask_img_filename Relative path to the mask image.
     */
    void use_mask_to_locate_walls(String mask_img_filename);

};

#endif
