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

#ifndef HEADER_LABYRINTH_H_INCLUDED
#define HEADER_LABYRINTH_H_INCLUDED

/**
 * @file labyrinth.h
 * @brief Functions to locate the labyrinth and detect walls.
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <laustracker/conf.h>
#include <laustracker/map.h>

using namespace std;
using namespace cv;

/// Contains information for a wall element
typedef struct wall_t
{
    /// Region of Interest
    Rect roi;
    /// Field index
    int x, y;
    /// Position within the field
    PosType pos;
    /// Is there a wall
    bool exists;

} wall_t;

/**
 * \brief Locate the labyrinth autonomously.
 * \param image Image of the labyrinth
 * \param hough_threshold Threshold for the Hough transformation
 * \param labyrinth_rect Reference of a std::vector that will contain the
 *        labyrinth position.
 * \param debuglevel Show debug information for intermediate steps.
 */
void findLabyrinth(const Mat& image, int hough_threshold,
                   vector< Point >& labyrinth_rect, int debuglevel = 0);

/**
 * \brief Locate the labyrinth autonomously via red markers at the labyrinth corners.
 * \param image Image of the labyrinth
 * \param labyrinth_rect Reference of a std::vector that will contain the
 *        labyrinth position.
 * \param labyrinth_conf_filename Configuration file that contains color values
 *        for the labyrinth markers.
 * \param debuglevel Show debug information for intermediate steps
 */
void findLabyrinth(const Mat& image, vector <Point>& labyrinth_rect,
                   String labyrinth_conf_filename, int debuglevel = 0);

/**
 * \brief Search for quads within a image.
 * \param image Image that should be searched
 * \param squares Reference of a vector of vectors with 4 elements each
 *        for the corners of the quads.
 * \param contours_img Binary image that will contain all quads that were found.
 * \param debuglevel Show debug information for intermediate steps
 */
void findSquares(const Mat& image, vector<vector<Point> >& squares,
                 Mat& contours_img, int debuglevel = 0);

/**
 * \brief Draw rectangles into a image
 * \param image Reference of a image to draw the rectangles.
 * \param squares  A vector of vectors with 4 elements each
 *        for the corners of the rectangles.
 */
void drawSquares(Mat& image, const vector<vector<Point> >& squares);

/**
 * \brief Locate bounded contours.
 * \param image Source and destination image
 */
void getContours(Mat& image);

/**
 * \brief Overlay the labyrinth with a grid for wall positions.
 * \param image Image of the labyrinth
 * \param labyrinth Position of the labyrinth within the image
 * \param walls Reference of a array with walls
 * \param debuglevel Show debug information for intermediate steps
 */
void getGrid(const Mat& image, const std::vector< Point >& labyrinth,
             vector< wall_t >& walls, int debuglevel = 0);

/**
 * \brief Overlay the labyrinth with a grid for wall positions from a mask image.
 * \param color_image Color image of the labyrinth to show debug images.
 * \param white_image Black-white image of the labyrinth to show debug images.
 * \param mask_filename Path to a mask image, relative to the program executable
 * \param labyrinth Position of the labyrinth within the image
 * \param walls Reference of a array with walls
 * \param debuglevel Show debug information for intermediate steps
 */
void getGrid2(const Mat& color_image, const Mat& white_image,
              const String mask_filename, const vector<Point> & labyrinth,
              vector <wall_t>& walls, int debuglevel);

/**
 * \brief Check a ROI (region of interest) whether there is a wall.
 * \param walls Reference of a array with walls
 * \param white_img Black-white image, only walls should be white
 * \param color_img Color image to show the results
 * \param debuglevel Show debug information for intermediate steps
 */
void check_for_wall(vector< wall_t >& walls, const Mat& white_img,
                    Mat& color_img, int debuglevel = 0);

/**
 * \brief Check a ROI (region of interest) whether there is a black line.
 * \param walls Reference of a array with walls
 * \param white_img Black-white image, only walls should be white
 * \param color_img Color image to show the results
 * \param debuglevel Show debug information for intermediate steps
 */
void check_for_black_line(vector< wall_t >& walls, const Mat& white_img,
                          Mat& color_img, int debuglevel = 0);

/**
 * \brief Draw the labyrinth rectangle into a image
 * \param image The source image
 * \param img The destination image
 * \param labyrinth Position of the labyrinth
 */
void drawLabyrinthRect(const Mat& image, Mat& img,
                       const vector<Point> & labyrinth);

#endif
