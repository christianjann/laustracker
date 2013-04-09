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

#include <laustracker/util.h>
#include <laustracker/image_manipulation.h>

#include <iostream>

ImageUndistort::ImageUndistort()
{

    set_intrinsics_xml("../share/laustracker/intrinsics.xml");
}

bool ImageUndistort::set_intrinsics_xml(string filename)
{
    filename = get_selfdir() + filename;

    cout << "Used intrinsics.xml: " << filename << "\n";

    fs = FileStorage();
    if (fs.open(filename, FileStorage::READ))
    {
        cout << "used: " << filename << "\n";
        size = Size((int)fs["image_width"], (int)fs["image_height"]);
        cout << "\nimage width: " << size.width;
        cout << "\nimage height: " << size.height;
        fs["camera_matrix"] >> intrinsic_matrix_loaded;
        fs["distortion_coefficients"] >> distortion_coeffs_loaded;
        cout << "\nintrinsic matrix:" << intrinsic_matrix_loaded;
        cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << endl;
        // Build the undistort map
        initUndistortRectifyMap(intrinsic_matrix_loaded, distortion_coeffs_loaded, Mat(),
                                intrinsic_matrix_loaded, size, CV_16SC2,
                                map1, map2);
        return true;
    }
    else
    {
        cout << "Warning: could not open intrinsics.xml\n";
        return false;
    }

}

ImageUndistort::~ImageUndistort()
{
}

void ImageUndistort::remap(const Mat& image, Mat &dst)
{
    if (!image.empty())
    {

        //OpenCV-2.3.1/modules/imgproc/src/imgwarp.cpp:2447:
        //error: (-215) dst.data != src.data in function remap
        Mat tmp;
        cv::remap(image, tmp, map1, map2, INTER_LINEAR, BORDER_CONSTANT, Scalar());
        dst = tmp;

    }

}

void white_regions(const Mat& image, Mat& white_regions, int thres)
{
    vector <Mat> planes;
    // the split function will separate the image into its channels
    // and store them in an vector of size 3
    split(image, planes);
    //cout << "size: " << planes.size() << " planes\n";
    //lautracker_imshow("blue",planes[0]);
    //lautracker_imshow("green",planes[1]);
    //lautracker_imshow("red",planes[2]);
    // separate white regions
    //Mat white_regions;
    bitwise_and(planes[0], planes[1], white_regions);
    bitwise_and(white_regions, planes[2], white_regions);
    // if (debug) laustracker_imshow("white_regions", white_regions);
    threshold(white_regions, white_regions, thres, 255, THRESH_BINARY);
    // if (debug) laustracker_imshow("white_regions_thres", white_regions);

    // alternative:
    //cvtColor(image, white_regions, COLOR_BGR2GRAY);
    //threshold(white_regions, white_regions, 150, 255, THRESH_BINARY);
}

void edm(const Mat& src, Mat& dst, int erode_size, int dilation_size, int morph_size, bool debug)
{
    if (erode_size > 0)
    {
        Mat erode_element = getStructuringElement(MORPH_RECT,
                            Size(2 * erode_size + 1, 2 * erode_size + 1),
                            Point(erode_size, erode_size));
        erode(src, dst, erode_element);
        if (debug) laustracker_imshow("edm_erode", dst);
    }

    if (dilation_size > 0)
    {
        Mat dilate_element = getStructuringElement(MORPH_RECT,
                             Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                             Point(dilation_size, dilation_size));
        dilate(dst, dst, dilate_element);

        if (debug) laustracker_imshow("edm_dilate", dst);
    }

    if (morph_size > 0)
    {
        int operation = 3; //Closing
        Mat morph_element =
            getStructuringElement(MORPH_ELLIPSE,
                                  Size(2 * morph_size + 1, 2 * morph_size + 1),
                                  Point(morph_size, morph_size));

        // Apply the specified morphology operation
        morphologyEx(dst, dst, operation, morph_element);

        if (debug) laustracker_imshow("edm_morph", dst);
    }
}

void get_thresholded_image(const Mat& image, Scalar min, Scalar max, Mat& dst, bool debug)
{
    cvtColor(image, dst, CV_BGR2HSV);
    if (debug) laustracker_imshow("get_thresholded_image:HSV", dst);
    inRange(dst, min, max, dst);
    if (debug) laustracker_imshow("get_thresholded_image:BW", dst);
}

// http://stackoverflow.com/questions/12204522/efficiently-threshold-red-using-hsv-in-opencv
void get_thresholded_image2(const Mat& image, Scalar min1, Scalar max1,
                            Scalar min2, Scalar max2, Mat& dst, bool debug)
{
    Mat tmp_img;
    cvtColor(image, tmp_img, CV_BGR2HSV);
    if (debug) laustracker_imshow("get_thresholded_image:HSV", tmp_img);
    inRange(tmp_img, min1, max1, dst);
    inRange(tmp_img, min2, max2, tmp_img);
    bitwise_or(tmp_img, dst, dst);
    if (debug) laustracker_imshow("get_thresholded_image:BW", dst);
}

void wrap_perspective(const Mat& src, Mat& dst, const vector <Point>& rect)
{
    Point2f srcQuad[] =
    {
        rect[0], //src Top left
        rect[1], // src Top right
        rect[2], // src Bottom right
        rect[3]  // src Bottom left
    };

    int width = rect[1].x - rect[0].x;
    int height = rect[3].y - rect[0].y;
    Point2f dstQuad[] =
    {
        rect[0],
        Point(rect[0].x + width, rect[0].y),
        Point(rect[0].x + width, rect[0].y + height),
        Point(rect[0].x, rect[0].y + height)
    };

    // COMPUTE PERSPECTIVE MATRIX
    Mat warp_mat = getPerspectiveTransform(srcQuad, dstQuad);
    Mat wraped;
    warpPerspective(src, wraped, warp_mat, src.size(),
                    INTER_LINEAR, BORDER_CONSTANT, Scalar());
    dst = wraped;
}

PerspectiveWrapper::PerspectiveWrapper()
{
    ready = false;
}

void PerspectiveWrapper::init(const vector <Point>& rect)
{
    objPts[0].x = rect[0].x; objPts[0].y = rect[0].y;
    objPts[1].x = rect[1].x; objPts[1].y = rect[1].y;
    objPts[2].x = rect[2].x; objPts[2].y = rect[2].y;
    objPts[3].x = rect[3].x; objPts[3].y = rect[3].y;

    width = rect[1].x - rect[0].x;
    height = rect[3].y - rect[0].y;

    imgPts[0].x = rect[0].x;         imgPts[0].y = rect[0].y;
    imgPts[1].x = rect[0].x + width; imgPts[1].y = rect[0].y;
    imgPts[2].x = rect[0].x + width; imgPts[2].y = rect[0].y + height;
    imgPts[3].x = rect[0].x;         imgPts[3].y = rect[0].y + height;

    //FIND THE HOMOGRAPHY
    H = getPerspectiveTransform(objPts, imgPts);
    ready = true;
}

void PerspectiveWrapper::wrap(const Mat& src, Mat& dst)
{
    if (ready)
    {
        Mat wraped;
        //USE HOMOGRAPHY TO REMAP THE IMAGE
        warpPerspective(src, wraped, H, src.size(), INTER_LINEAR,
                        BORDER_CONSTANT, Scalar::all(0));
        dst = wraped;
    }
    else
    {
        cout << "Error: you need to init the wrapper first\n";
    }
}

vector <Point> PerspectiveWrapper::get_rect()
{
    vector <Point> pts;
    pts.push_back(Point(imgPts[0].x, imgPts[0].y));
    pts.push_back(Point(imgPts[1].x, imgPts[1].y));
    pts.push_back(Point(imgPts[2].x, imgPts[2].y));
    pts.push_back(Point(imgPts[3].x, imgPts[3].y));

    return pts;
}
