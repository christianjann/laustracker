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

#ifndef HEADER_LAUSTRACKER_IMAGE_MANIPULATION_H_INCLUDED
#define HEADER_LAUSTRACKER_IMAGE_MANIPULATION_H_INCLUDED

/**
 * @file image_manipulation.h
 * @brief Contains functions to manipulate images
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

/**
 * @brief Class to undistort images
 *
 */
class ImageUndistort
{
private:
    /// To read the intrinsics_xml file
    FileStorage fs;
    Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
    /// The undistort map
    Mat map1, map2;
    /// Image size
    Size size;

public:
    /**
     * \brief Constructor
     *
     * \param file_intrinsics_xml Path for the intrinsics.xml file,
     *        that contains the camera calibration values
     *
     */
    ImageUndistort();

    /**
     * \brief Deconstructor
     *
     */
    ~ImageUndistort();

    /**
     * \brief Undistort a image.
     *
     * \param image The distorted image
     *
     * \param dst The undistorted image
     *
     */
    void remap(const Mat& image, Mat &dst);

    /**
     * \brief Load a intrinsics.xml file
     * \param filename Relative path for the intrinsics.xml file.
     * \return true if the file has been found.
     */
    bool set_intrinsics_xml(string filename);
};

/**
 * @brief Calculate a binary black-white image.
 *
 * \param image The source image
 * \param white_regions The black-white image
 * \param thres Threshold, the smaller, the more white regions you will get
 */
void white_regions(const Mat& image, Mat& white_regions, int thres = 100);

/**
 * @brief Erode, Dilate, Morph
 *
 * \param src The source image
 * \param dst The destination image
 * \param erode_size The size of the structuring element
 *        for the erode operation.
 * \param dilation_size The size of the structuring element
 *        for the dilate operation.
 * \param morph_size The size of the structuring element
 *        for the morphology operation.
 * \param debug Show debug images for intermediate steps?
 */
void edm(const Mat& src, Mat& dst, int erode_size = 3, int dilation_size = 4,
         int morph_size = 10, bool debug = false);
/**
 * \brief Create a thresholded image with white regions
 * \param image The source image
 * \param min Lower boundary
 * \param max Upper boundary
 * \param dst The destination image
 * \param debug Show debug images for intermediate steps?
 */
void get_thresholded_image(const Mat& image, Scalar min, Scalar max, Mat& dst,
                           bool debug = false);

/**
 * \brief Create a thresholded image with white regions from two color ranges
 *
 * See also http://stackoverflow.com/questions/12204522/efficiently-threshold-red-using-hsv-in-opencv
 * \param image The source image
 * \param min1 First lower boundary
 * \param max1 First upper boundary
 * \param min2 Second lower boundary
 * \param max2 Second upper boundary
 * \param dst The destination image
 * \param debug Show debug images for intermediate steps?
 */
void get_thresholded_image2(const Mat& image, Scalar min1, Scalar max1,
                            Scalar min2, Scalar max2, Mat& dst, bool debug = false);

/**
 * \brief Wrap the perspective
 * \param src The source image
 * \param dst The destination image
 * \param rect Warped rect to unwrap
 */
void wrap_perspective(const Mat& src, Mat& dst, const vector <Point>& rect);

/**
 * @brief A class to do a perspective transformation.
 *
 */
class PerspectiveWrapper
{
private:
    /// Width and height of the rect
    int width, height;
    /// Object and image points
    Point2f objPts[4], imgPts[4];
    /// THE HOMOGRAPHY
    Mat H;
    /// Was the init() function already called?
    bool ready;
public:
    /// Constructor
    PerspectiveWrapper();
    /**
     * \brief Initialization
     * \param rect A wrapped rect that should be unwrapped
     */
    void init(const vector <Point>& rect);
    /**
     * \brief Wrap the perspective
     * \param src The original image
     * \param dst The destination image
     */
    void wrap(const Mat& src, Mat& dst);

    /**
     * \brief Returns the unwrapped rect.
     * \return The unwrapped rect
     */
    vector <Point> get_rect();
};
#endif
