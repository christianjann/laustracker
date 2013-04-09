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

#ifndef HEADER_LAUSTRACKER_UTIL_H_INCLUDED
#define HEADER_LAUSTRACKER_UTIL_H_INCLUDED

/**
 * @file util.h
 * @brief Small helper functions.
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <iterator>
#include <vector>
#include <boost/property_tree/ptree.hpp>

using namespace cv;
using namespace std;

/**
 * \brief Like imshow(), resizes the image on smaller screens.
 *
 * \param windoname Window title
 * \param image Image to show
 */
void laustracker_imshow(const string& windoname, const Mat& image);

/**
 * \brief Like imshow(), resizes the image on smaller screens and crops
 *        the image if the labyrinth position is known.
 *
 * \param windoname Window title
 * \param image Image to show
 */
void laustracker_imshow_croped(const string& windoname, const Mat& image,
                               vector <Point> rect);

/**
 * \brief Calculate the cosine between two vectors.
 *
 * The angle gets spanned from pt0 to pt1 and from pt0 to pt2.
 *
 * \param pt0 Origin
 * \param pt1 First point
 * \param pt2 Second point
 */
double angle(Point pt1, Point pt2, Point pt0);


/**
 * \brief Calculate the intersection point of two straight lines.
 *
 * \param o1 First point of the first straight line.
 * \param p1 Second point of the first straight line.
 * \param o2 First point of the second straight line.
 * \param p2 Second point of the second straight line.
 * \param r Reference of a Point to store the intersection.
 * \return false, if there is no intersection
 */
bool intersection(Point o1, Point p1, Point o2, Point p2, Point &r);

/**
 * \brief Scale a rectangle.
 *
 * \param rect Vector of four points
 * \param pixels Number of pixels to increase/decrease the rectangle.
 */
void scale_rect(vector <Point>& rect, int pixels = 15);


/**
 * \brief Count the number of pixels of a specified color within a image region.
 *
 *  @code Mat roi = white_img(rois[i]); @endcode
 *  @code int whitecount = countColor(roi, 255);@endcode
 *
 * \param image Image or ROI
 * \param color Color: 0 for black, 255 for white
 */
int countColor(cv::Mat image, int color = 255);

/**
 * \brief Get the path of the current executable
 * \return Name of the executable with path
 */
string get_selfpath();

/**
 * \brief Determine the directory of the current executable.
 * \param debug Print the path to stdout.
 * \return Directory of the current executable.
 */
string get_selfdir(bool debug = false);

/**
 * \brief Calculate the dot product of three points
 *
 */
float DotProductPoints(Point2f const &a, Point2f const &b, Point2f const &c);

/**
 * \brief Calculate the cross product of three points
 *
 */
float CrossProductPoints(Point2f const &a, Point2f const &b, Point2f const &c);

/**
 * \brief Calculate the distance between two points
 * \param a Point A
 * \param b Point B
 * \return Distance
 */
float DistancePointPoint(Point2f const &a, Point2f const &b);

/**
 * \brief Calculate the Distance between a straight line a a point
 *
 */
float DistanceLinePoint(Point2f const &a, Point2f const &b, Point2f const &c, bool isSegment);

/**
 * \brief Calculate the center point between two points
 * \param a Point A
 * \param b Point B
 * \return Center point
 */
Point2f centerPointPoint(Point2f a, Point2f b);

/**
 * \brief Convert radians to degrees
 * \param radians Angle in radians
 * \return Angle in degrees
 */
float degrees(float radians);

template<typename T>
std::vector<T> to_array(const std::string& s)
{
    std::vector<T> result;
    std::stringstream ss(s);
    std::string item;
    while (ss >> item)
    {
        //cout << "item: " << item<<"\n";
        result.push_back(boost::lexical_cast<T>(item));
    }
    return result;
}

template<typename T>
std::ostream &operator <<(std::ostream &os, const std::vector<T> &v)
{
    using namespace std;
    copy(v.begin(), v.end(), ostream_iterator<T>(os, "\n"));
    return os;
}

/**
 * @brief Class to measure frames per second
 *
 */
class FpS
{

private:

    int64_t timespecDiff(struct timespec *timeA_p, struct timespec *timeB_p);
    int fps_counter;
    struct timespec fps_time_start, fps_time_end;

public:

    /**
     * \brief Constructor.
     * Get the first measurement
     */
    FpS();

    /// Current frame rate
    float fps;

    /// Actualize the frame rate
    void measure();

    /// Print the frame rate
    void print();

    /// The frame rate has been updated
    bool isNew();

};

/**
 * \brief Get a HSV color from a Boost property_tree and convert
 *        the color from Gimp's color space into OpenCV's color space
 * \param conf Boost property_tree
 * \param color_name Name of the color to read
 * \return cv::Scalar with HSV color
 */
Scalar get_gimp_color(boost::property_tree::ptree conf, string color_name);



#endif
