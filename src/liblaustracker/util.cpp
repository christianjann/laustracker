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

// some functions are from cvBlob

// Copyright (C) 2007 by Cristóbal Carnero Liñán
// grendel.ccl@gmail.com
//
// cvBlob is free software: you can redistribute it and/or modify
// it under the terms of the Lesser GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// cvBlob is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// Lesser GNU General Public License for more details.
//
// You should have received a copy of the Lesser GNU General Public License
// along with cvBlob.  If not, see <http://www.gnu.org/licenses/>.

#include <laustracker/util.h>
#include <iostream>
#include <cmath>
#include <unistd.h> // readlink()
#include <time.h> // framerate

void laustracker_imshow(const string& windoname, const Mat& image)
{
    //TODO Maybe add a number to each window title
    float maxheight = 700;
    if (image.size().height > maxheight)
    {

        float scale = maxheight / image.size().height;
        Mat view;
        resize(image, view, Size(), scale, scale); // 1/2 resizing
        imshow(windoname, view);

    }
    else
    {
        imshow(windoname, image);
    }
}

void laustracker_imshow_croped(const string& windoname, const Mat& image,
                               vector <Point> rect)
{
    vector <Point> tmp_rect = rect;
    scale_rect(tmp_rect, 30);
    Rect roi = boundingRect(tmp_rect);

    laustracker_imshow(windoname, image(roi));
}

// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle(Point pt1, Point pt2, Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) *
                                          (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool intersection(Point o1, Point p1, Point o2, Point p2,
                  Point &r)
{
    Point x = o2 - o1;
    Point d1 = p1 - o1;
    Point d2 = p2 - o2;

    float cross = d1.x * d2.y - d1.y * d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x) / cross;
    r = o1 + d1 * t1;
    return true;
}


void scale_rect(vector <Point>& rect, int pixels)
{
    if (rect.size() == 4)
    {
        rect[0].x -= pixels;
        rect[0].y -= pixels;

        rect[1].x += pixels;
        rect[1].y -= pixels;

        rect[2].x += pixels;
        rect[2].y += pixels;

        rect[3].x -= pixels;
        rect[3].y += pixels;
    }
    else
        cout << "rect has wrong size\n";
}

int countColor(cv::Mat image, int color)
{
    int counter = 0;
    // obtain iterator at initial position
    cv::Mat_<cv::Vec3b>::iterator it = image.begin<cv::Vec3b>();
    // obtain end position
    cv::Mat_<cv::Vec3b>::iterator itend = image.end<cv::Vec3b>();
    // loop over all pixels
    for (; it != itend; ++it)
    {
        if ((*it)[0] == color && (*it)[1] == color && (*it)[2] == color)
            counter++;
    }
    return counter;
}

string get_selfpath()
{
    char buff[1024];
    // "/proc/self/exe" is Linux-specific
    ssize_t len = readlink("/proc/self/exe", buff, sizeof(buff) - 1);
    if (len != -1)
    {
        buff[len] = '\0';
        return std::string(buff);
    }
    else
    {
        /* handle error condition */
    }
}

string get_selfdir(bool debug)
{
    string path = get_selfpath();
    if (debug) cout << "Executable Path: " << path << "\n";
    string dir = path.substr(0, path.find_last_of('/') + 1);
    if (debug) cout << "Executable Dir: " << dir << "\n";
    return dir;
}

// https://community.topcoder.com/tc?module=Static&d1=tutorials&d2=geometry1
float DotProductPoints(Point2f const &a, Point2f const &b, Point2f const &c)
{
    float abx = b.x - a.x;
    float aby = b.y - a.y;
    float bcx = c.x - b.x;
    float bcy = c.y - b.y;

    return abx * bcx + aby * bcy;
}

float CrossProductPoints(Point2f const &a, Point2f const &b, Point2f const &c)
{
    float abx = b.x - a.x;
    float aby = b.y - a.y;
    float acx = c.x - a.x;
    float acy = c.y - a.y;

    return abx * acy - aby * acx;
}

float DistancePointPoint(Point2f const &a, Point2f const &b)
{
    float abx = a.x - b.x;
    float aby = a.y - b.y;

    return sqrt(abx * abx + aby * aby);
}

float DistanceLinePoint(Point2f const &a, Point2f const &b, Point2f const &c,
                        bool isSegment)
{
    if (isSegment)
    {
        float dot1 = DotProductPoints(a, b, c);
        if (dot1 > 0) return DistancePointPoint(b, c);

        float dot2 = DotProductPoints(b, a, c);
        if (dot2 > 0) return DistancePointPoint(a, c);
    }

    //return fabs(CrossProductPoints(a, b, c) / DistancePointPoint(a, b));
    return CrossProductPoints(a, b, c) / DistancePointPoint(a, b);
}

Point2f centerPointPoint(Point2f a, Point2f b)
{
    Point2f ret;
    ret.x = 0.5 * (a.x + b.x);
    ret.y = 0.5 * (a.y + b.y);
    return ret;
}

float degrees(float radians)
{
    return radians * 180 / 3.1415926;
}

FpS::FpS()
{
    fps_counter = 0;
    clock_gettime(CLOCK_MONOTONIC, &fps_time_start);
    float fps = 0;

}

int64_t FpS::timespecDiff(struct timespec *timeA_p, struct timespec *timeB_p)
{
    return ((timeA_p->tv_sec * 1000000000) + timeA_p->tv_nsec) -
           ((timeB_p->tv_sec * 1000000000) + timeB_p->tv_nsec);
}

void FpS::measure()
{
    fps_counter++;
    if (fps_counter == 10)
    {
        clock_gettime(CLOCK_MONOTONIC, &fps_time_end);
        uint64_t timeElapsed = timespecDiff(&fps_time_end, &fps_time_start);
        fps = (double)1000000000 / (double)timeElapsed * fps_counter;
        clock_gettime(CLOCK_MONOTONIC, &fps_time_start);
        fps_counter = 0;
    }

}

bool FpS::isNew()
{
    return fps_counter == 0;
}

void FpS::print()
{
    if (isNew())
    {
        cout << "FPS: ";
        cout.precision(4);
        cout << fps << "\n";
    }

}

Scalar get_gimp_color(boost::property_tree::ptree conf, string color_name)
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
    vector<int> col = to_array<int>(conf.get<std::string>(color_name));


    return  Scalar(col[0] * (179.0 / 360),
                   col[1] * (255.0 / 100),
                   col[2] * (255.0 / 100));
}
