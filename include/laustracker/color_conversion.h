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

/* Copyright (c) 2011 the authors listed at the following URL, and/or
 * the authors of referenced articles or incorporated external code:
 * http://en.literateprograms.org/RGB_to_HSV_color_space_conversion_(C)\
 * ?action=history&offset=20110802173944
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Retrieved from:
 * http://en.literateprograms.org/RGB_to_HSV_color_space_conversion_(C)?oldid=17206
 */

#ifndef HEADER_LAUSTRACKER_COLOR_CONVERSATION_H_INCLUDED
#define HEADER_LAUSTRACKER_COLOR_CONVERSATION_H_INCLUDED

/**
 * @file color_conversion.h
 * @brief Contains functions to convert colors between different color spaces.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>

using namespace cv;
/// Calculate the minimum of 3 values
#define MIN3(x,y,z)  ((y) <= (z) ? \
                      ((x) <= (y) ? (x) : (y)) \
                          : \
                          ((x) <= (z) ? (x) : (z)))
/// Calculate the maximum of 3 values
#define MAX3(x,y,z)  ((y) >= (z) ? \
                      ((x) >= (y) ? (x) : (y)) \
                          : \
                          ((x) >= (z) ? (x) : (z)))

/// Contains a RGB color.
struct rgb_color
{
    /// Value for red
    unsigned char r; /* Channel intensities between 0 and 255 */

    /// Value for green
    unsigned char g;

    /// Value for blue
    unsigned char b;
};

/// Contains a HSV color
struct hsv_color
{
    /// Hue degree between 0 and 255
    unsigned char hue;
    /// Saturation between 0 (gray) and 255
    unsigned char sat;
    /// Value between 0 (black) and 255
    unsigned char val;
};

/**
 * \brief Convert a color from RGB to HSV
 * \param rgb_color RGB color
 * \return HSV color
 */
struct hsv_color rgb_to_hsv(struct rgb_color rgb);


/**
 * \brief Convert a color from RGB to HSV
 *
 * HSV colors
 * OpenCV:
 *       H: 0..179
 *       S: 0..255
 *       V: 0..255
 *
 * \param rgb RGB color, coded as BGR, first blue, then green and the last is red.
 * \return HSV color
 */
Scalar bgr_to_cv_hsv(Scalar rgb);

/**
 * \brief Convert a color from RGB to HSV (Gimp)
 *
 * HSV colors
 * Gimp:
 *       H: 0..360
 *       S: 0..100
 *       V: 0..100
 *
 * \param rgb RGB color, coded as BGR, first blue, then green and the last is red.
 * \return HSV color
 */
Scalar bgr_to_gimp_hsv(Scalar rgb);


/**
 * \brief Convert a color from RGB to HSV (KColorChooser)
 * HSV colors
 * KColorChooser:
 *       H: 0..359
 *       S: 0..255
 *       V: 0..255
 *
 * \param rgb RGB color, coded as BGR, first blue, then green and the last is red.
 * \return HSV color
 */
Scalar bgr_to_kcc_hsv(Scalar rgb);

#endif
