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

#include <laustracker/labyrinth.h>
#include <laustracker/util.h>
#include <laustracker/image_manipulation.h>
#include <list>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

void drawLabyrinthRect(const Mat& image, Mat& img, const vector<Point> & labyrinth)
{
    img = image.clone();
    const Point* p = &labyrinth[0];
    int n = (int)labyrinth.size();
    polylines(img, &p, &n, 1, true, Scalar(255, 0, 0), 2, CV_AA);

}

void findLabyrinth(const Mat& image, int hough_threshold,
                   vector <Point>& labyrinth_rect, int debuglevel)
{

    Mat edges, src_gray(image), standard_hough;
    /// Pass the image to gray
    cvtColor(src_gray, src_gray, CV_RGB2GRAY);

    /// Apply Canny edge detector
    Canny(src_gray, edges, 50, 200, 3);

    vector<Vec2f> s_lines;
    cvtColor(edges, standard_hough, CV_GRAY2BGR);

    /// 1. Use Standard Hough Transform
    HoughLines(edges, s_lines, 1, CV_PI / 180, hough_threshold, 0, 0);

    Point upper_boundary[2], lower_boundary[2], right_boundary[2], left_boundary[2];
    for (int i = 0; i < 2; i++)
    {
        upper_boundary[i] = Point(image.size().width / 2, image.size().height / 2);
        lower_boundary[i] = Point(image.size().width / 2, image.size().height / 2);
        right_boundary[i] = Point(image.size().width / 2, image.size().height / 2);
        left_boundary[i] = Point(image.size().width / 2, image.size().height / 2);
    }


    /// Show the result
    for (int i = 0; i < s_lines.size(); i++)
    {
        float roh = s_lines[i][0], theta = s_lines[i][1];
        float degrees = theta * 180 / (3.1412);
        //cout << "hough r: " << roh << "\t t: " << theta
        //cout << "\tdegrees: " << degrees << " \n";
        double cos_t = cos(theta), sin_t = sin(theta);
        double x0 = roh * cos_t, y0 = roh * sin_t;
        double alpha = 1000;

        Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
        Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));

        if (degrees < 6 || degrees > 174 && degrees < 186) // vertical line
        {
            line(standard_hough, pt1, pt2, Scalar(255, 0, 0), 3, CV_AA);
            // line is nearer at left
            if (pt1.x < left_boundary[0].x && pt2.x < left_boundary[1].x)
            {
                left_boundary[0] = pt1;
                left_boundary[1] = pt2;
            }
            // line is nearer at right
            if (pt1.x > right_boundary[0].x && pt2.x > right_boundary[1].x)
            {
                right_boundary[0] = pt1;
                right_boundary[1] = pt2;
            }
        }
        else if (degrees > 84 && degrees < 96) // horizontal line
        {
            line(standard_hough, pt1, pt2, Scalar(0, 255, 0), 3, CV_AA);
            // line is nearer at top
            if (pt1.y < upper_boundary[0].y && pt2.y < upper_boundary[1].y)
            {
                upper_boundary[0] = pt1;
                upper_boundary[1] = pt2;
            }
            // line nearer at bottom
            if (pt1.y > lower_boundary[0].y && pt2.y > lower_boundary[1].y)
            {
                lower_boundary[0] = pt1;
                lower_boundary[1] = pt2;
            }
        }
        else
            line(standard_hough, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);

    }

    line(standard_hough, left_boundary[0], left_boundary[1],
         Scalar(255, 255, 255), 3, CV_AA);
    line(standard_hough, right_boundary[0], right_boundary[1],
         Scalar(255, 255, 255), 3, CV_AA);
    line(standard_hough, upper_boundary[0], upper_boundary[1],
         Scalar(255, 255, 255), 3, CV_AA);
    line(standard_hough, lower_boundary[0], lower_boundary[1],
         Scalar(255, 255, 255), 3, CV_AA);

    // get the edge points
    Point edge;
    //vector <Point> edge_points;

    intersection(left_boundary[0], left_boundary[1],
                 upper_boundary[0], upper_boundary[1], edge);
    circle(standard_hough, edge, 10, Scalar(0, 0, 255), 5);
    labyrinth_rect.push_back(edge);

    intersection(right_boundary[0], right_boundary[1],
                 upper_boundary[0], upper_boundary[1], edge);
    circle(standard_hough, edge, 10, Scalar(0, 0, 255), 5);
    labyrinth_rect.push_back(edge);

    intersection(right_boundary[0], right_boundary[1],
                 lower_boundary[0], lower_boundary[1], edge);
    circle(standard_hough, edge, 10, Scalar(0, 0, 255), 5);
    labyrinth_rect.push_back(edge);

    intersection(left_boundary[0], left_boundary[1],
                 lower_boundary[0], lower_boundary[1], edge);
    circle(standard_hough, edge, 10, Scalar(0, 0, 255), 5);
    labyrinth_rect.push_back(edge);


    if (debuglevel > 3) laustracker_imshow("cv_hough_lines", standard_hough);
    //imwrite( "cv_hough_lines+edges.jpg", standard_hough );
}

typedef struct edge_t
{
    /// Position im Bild (in Pixeln)
    Point2f center;
    /// Abstand zur Bildmitte
    float distance;
} edge_t;


bool compare_distance(edge_t first, edge_t second)
{
    if (first.distance < second.distance) return true;
    else return false;
}

bool compare_x(Point first, Point second)
{
    if (first.x < second.x) return true;
    else return false;
}

bool compare_y(Point first, Point second)
{
    if (first.y < second.y) return true;
    else return false;
}

void findLabyrinth(const Mat& image, vector <Point>& labyrinth_rect,
                   String labyrinth_conf_filename, int debuglevel)
{
    Mat red, white, color_img;

    cout << "locate the labyrinth via red dots\n";

    /// Thresholds for the red markers at the labyrinth corners
    Scalar redmin1, redmax1, redmin2, redmax2;

    boost::property_tree::ptree labyrinth_conf;
    try
    {
        cout << "Trying to open " << labyrinth_conf_filename << "\n";
        boost::property_tree::ini_parser::read_ini(
            get_selfdir(debuglevel > 2) + labyrinth_conf_filename, labyrinth_conf);
    }
    catch (...)
    {
        cout << "Warning: could not open " << labyrinth_conf_filename << "\n";
    }

    redmin1 = get_gimp_color(labyrinth_conf, "red_dots.hsv_redmin_1");
    redmax1 = get_gimp_color(labyrinth_conf, "red_dots.hsv_redmax_1");
    redmin2 = get_gimp_color(labyrinth_conf, "red_dots.hsv_redmin_2");
    redmax2 = get_gimp_color(labyrinth_conf, "red_dots.hsv_redmax_2");

    color_img = image.clone();
    get_thresholded_image2(color_img, redmin1, redmax1, redmin2, redmax2, red);
    //GaussianBlur(red, red, Size(9, 9), 2, 2 );
    edm(red, red, 1, 10, 10);

    if (debuglevel > 3) laustracker_imshow("locate_labyrinth_reddots", color_img);
    if (debuglevel > 3) laustracker_imshow("cv_labyrinth_red_dots_thres", red);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Size image_size = image.size();
    if (debuglevel > 2) cout << "image_width: " << image_size.width
                                 << " image_height: " << image_size.height << "\n";
    Point2f image_center(image_size.width / 2.0, image_size.height / 2.0);
    int center_distance = 99999;
    list<Point> edge_dots, top_dots, bottom_dots;
    vector<Point> red_dots;

    findContours(red, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    for (int n = 0; n < (int)contours.size(); n++)
    {
        float radius;
        Point2f center;
        minEnclosingCircle(contours[n], center , radius);
        red_dots.push_back(center);
    }

    if (red_dots.size() >= 4)
    {
        if (debuglevel > 2) cout << "> 4\n";

        Mat rects_img, rects_img2;
        if (debuglevel > 3)
        {
            rects_img = color_img.clone();
            rects_img2 = color_img.clone();
        }

        // get all combinations of four points
        // http://stackoverflow.com/questions/127704/algorithm-to-return-all-combinations-of-k-elements-from-n
        // http://bytes.com/topic/c/answers/169974-c-algorithm-combinations-vector-elements
        // :-) http://stackoverflow.com/questions/9430568/generating-combinations-in-c
        int comb_size = 4;
        vector<bool> v(red_dots.size());
        fill(v.begin() + comb_size, v.end(), true);
        do
        {
            vector <Point> rect;
            for (int i = 0; i < red_dots.size(); ++i)
            {
                if (!v[i])
                {
                    if (debuglevel > 4) cout << (red_dots.at(i)) << "\n";
                    rect.push_back(red_dots.at(i));
                }
            }

            // if (debuglevel > 2) cout << (rect) << "\n";

            top_dots.clear();
            bottom_dots.clear();

            list <Point> lab_rect(rect.begin(), rect.end());
            lab_rect.sort(compare_y);

            top_dots.push_back(lab_rect.front());
            lab_rect.pop_front();
            top_dots.push_back(lab_rect.front());
            lab_rect.pop_front();

            bottom_dots.push_back(lab_rect.front());
            lab_rect.pop_front();
            bottom_dots.push_back(lab_rect.front());

            top_dots.sort(compare_x);
            bottom_dots.sort(compare_x);

            rect.clear();

            rect.push_back(Point(top_dots.front().x, top_dots.front().y));
            rect.push_back(Point(top_dots.back().x, top_dots.back().y));
            rect.push_back(Point(bottom_dots.back().x, bottom_dots.back().y));
            rect.push_back(Point(bottom_dots.front().x, bottom_dots.front().y));

            if (debuglevel > 4)
            {
                drawLabyrinthRect(rects_img, rects_img, rect);
                laustracker_imshow("locate_labyrinth_reddots_all_rects", rects_img);

                //drawLabyrinthRect(color_img, rects_img, rect);
                //laustracker_imshow("locate_labyrinth_reddots_current_rect", rects_img);
                //waitKey();
            }

            if (fabs(contourArea(Mat(rect))) > 0.2 * image.size().width * image.size().height &&
                    isContourConvex(Mat(rect)))
            {

                double maxCosine = 0;

                for (int j = 2; j < 5; j++)
                {
                    // find the maximum cosine of the angle between joint edges
                    double cosine = fabs(angle(rect[j % 4], rect[j - 2], rect[j - 1]));
                    maxCosine = MAX(maxCosine, cosine);
                }
                if (debuglevel > 2) cout << "maxCosine: " << maxCosine << "\n";

                // if cosines of all angles are small
                // (all angles are ~90 degree)
                if (maxCosine < 0.1)
                {
                    // test if it is a quad, side lengths should be nearly equal
                    int n10 = norm(rect[1] - rect[0]);
                    int n21 = norm(rect[2] - rect[1]);

                    // the difference of two side lengths should not be bigger
                    // than 50% of one side length
                    if (abs(n10 - n21) < 0.5 * n10 && abs(n10 - n21) < 0.5 * n21)
                    {
                        // find the smallest rectangle with the smallest
                        // distance to the image center
                        int dist = DistancePointPoint(rect[0], image_center)
                                   + DistancePointPoint(rect[1], image_center)
                                   + DistancePointPoint(rect[2], image_center)
                                   + DistancePointPoint(rect[3], image_center);
                        if (dist < center_distance)
                        {
                            center_distance = dist;
                            labyrinth_rect = rect;

                            cout << "labyrinth found (maxCosine: " << maxCosine << ")\n";

                            if (debuglevel > 3)
                            {
                                drawLabyrinthRect(rects_img2, rects_img2, labyrinth_rect);
                                laustracker_imshow("locate_labyrinth_reddots_filtered_rects",
                                                   rects_img2);
                                waitKey(1000);
                            }
                        }
                    }
                }
            }

            //if (debuglevel > 2) cout << "\n";
        }
        while (next_permutation(v.begin(), v.end()));

        if (debuglevel > 3)
        {
            Mat rects_img3;
            rects_img3 = color_img.clone();
            drawLabyrinthRect(rects_img3, rects_img3, labyrinth_rect);
            laustracker_imshow("locate_labyrinth_reddots_rects_endresult", rects_img3);
        }
    }

    else
    {
        cout << "Error: 4 red dots not found\n";
    }

}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
void findSquares(const Mat& image, vector<vector<Point> >& squares,
                 Mat& contours_img, int debuglevel)
{
    squares.clear();

    Mat gray, canny_output, dilate_output, debug_img;
    if (debuglevel > 3) debug_img = image.clone();
    int thresh = 25;
    cvtColor(image, gray, CV_BGR2GRAY);
    blur(gray, gray, Size(3, 3));
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Detect edges using canny
    Canny(gray, canny_output, thresh, thresh * 2, 3);
    if (debuglevel > 3) laustracker_imshow("cv_canny", canny_output);

    int dilation_size = 3;
    Mat element =
        getStructuringElement(MORPH_RECT, Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                              Point(dilation_size, dilation_size));
    // Apply the dilation operation
    dilate(canny_output, dilate_output, element);
    if (debuglevel > 3) laustracker_imshow("cv_dilate", dilate_output);

    // Find contours
    findContours(dilate_output, contours, hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Draw contours
    contours_img = Mat::zeros(image.size(), CV_8UC3);


    vector<Point> approx;

    // test each contour
    for (size_t i = 0; i < contours.size(); i++)
    {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        approxPolyDP(Mat(contours[i]), approx,
                     arcLength(Mat(contours[i]), true) * 0.02, true);

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation

        if (approx.size() == 4 &&
                fabs(contourArea(Mat(approx))) > 1000 &&
                isContourConvex(Mat(approx)))
        {

            double maxCosine = 0;

            for (int j = 2; j < 5; j++)
            {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                maxCosine = MAX(maxCosine, cosine);
            }

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if (maxCosine < 0.3)
            {
                // test if it is a quad, side lengths should be nearly equal
                int n10 = norm(approx[1] - approx[0]);
                int n21 = norm(approx[2] - approx[1]);
                //cout<<"norm 1-0 "<<n10<<"\n";
                //cout<<"norm 2-1 "<<n21<<"\n";

                //draw the two sides that were used for the calculation
                //line(contours_img, approx[0], approx[1], Scalar(0, 0, 255), 3);
                //line(contours_img, approx[1], approx[2], Scalar(0, 255, 0), 3);

                // the difference of two side lengths should not be bigger
                // than 50% of one side length
                if (abs(n10 - n21) < n10 && abs(n10 - n21) < n21)
                {
                    squares.push_back(approx);

                    drawContours(contours_img, contours, i,
                                 Scalar(255, 255, 255), 2, 8);

                    if (debuglevel > 3) drawContours(debug_img, contours, i,
                                                         Scalar(0, 255, 0), 2, 8);
                }
            }
        }
    }

    if (debuglevel > 3)
    {
        laustracker_imshow("cv_filtered_contours", contours_img);
        laustracker_imshow("cv_filtered_contours_lab", debug_img);
    }
    //imwrite("cv_filtered_contours.jpg",contours_img);

}


// the function draws all the squares in the image
void drawSquares(Mat& image, const vector<vector<Point> >& squares)
{
    for (size_t i = 0; i < squares.size(); i++)
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0, 255, 0), 2, CV_AA);
    }

    laustracker_imshow("cv_Square_Detection", image);
    //imwrite("cv_Square_Detection.jpg", image);
}

void getContours(Mat& image)
{
    int thresh = 100;
    cvtColor(image, image, CV_BGR2GRAY);
    blur(image, image, Size(3, 3));
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Detect edges using canny
    Canny(image, canny_output, thresh, thresh * 2, 3);
    // Find contours
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Draw contours
    image = Mat::zeros(canny_output.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(255, 255, 255);
        drawContours(image, contours, i, color, 2, 8, hierarchy, 0, Point());
    }

    //cvtColor( image, image, CV_BGR2GRAY );
}



void getGrid(const Mat& image, const vector<Point> & labyrinth,
             vector <wall_t>& walls, int debuglevel)
{
    Point top_left = labyrinth[0];
    Point top_right = labyrinth[1];
    Point bottom_right = labyrinth[2];
    Point bottom_left = labyrinth[3];

    cout << "Point top_left" << labyrinth[0] << "\n";
    cout << "Point top_right" << labyrinth[1] << "\n";
    cout << "Point bottom_right" << labyrinth[2] << "\n";
    cout << "Point bottom_left" << labyrinth[3] << "\n";

    //line(image,labyrinth[0],labyrinth[1],Scalar(0,0,255),1);
    int fieldwidth_top = (top_right.x - top_left.x) / FIELD_COUNT;
    int fieldwidth_bottom = (bottom_right.x - bottom_left.x) / FIELD_COUNT;
    int fieldheight_right = (bottom_right.y - top_right.y) / FIELD_COUNT;
    int fieldheight_left = (bottom_left.y - top_left.y) / FIELD_COUNT;

    cout << "Field width top: " << fieldwidth_top << " pixel\n";
    cout << "Field with bottom: " << fieldwidth_bottom << " pixel\n";
    cout << "Field height left: " << fieldheight_left << " pixel\n";
    cout << "Field height right: " << fieldheight_right << " pixel\n";



    Mat img = image.clone();
    //drawLabyrinthRect(img, labyrinth);
    for (int i = 0; i < FIELD_COUNT + 1; i++)
    {
        Point p1, p2;
        p1.x = top_left.x;
        p1.y = top_left.y + i * fieldheight_left;
        p2.x = top_right.x;
        p2.y = top_right.y + i * fieldheight_right;


        line(img, p1, p2, Scalar(0, 255, 0), 4);
    }

    for (int i = 0; i < FIELD_COUNT + 1; i++)
    {
        Point p1, p2;
        p1.x = top_left.x + i * fieldwidth_top;
        p1.y = top_left.y;
        p2.x = bottom_left.x + i * fieldwidth_bottom;
        p2.y = bottom_left.y;


        line(img, p1, p2, Scalar(0, 255, 0), 4);
    }

    Point p1, p2, p3, p4;
    for (int i = 0; i <= FIELD_COUNT; i++)
    {
        for (int j = 0; j <= FIELD_COUNT; j++)
        {

            for (int type = 0; type < 2; type++)
            {
                bool run = false;
                if (type == 0 && j != FIELD_COUNT)
                {
                    p1.x = top_left.x;
                    p1.y = top_left.y + i * fieldheight_left;
                    p2.x = top_right.x;
                    p2.y = top_right.y + i * fieldheight_right;

                    p3.x = top_left.x + j * fieldwidth_top + fieldwidth_top / 2;
                    p3.y = top_left.y;
                    p4.x = bottom_left.x + j * fieldwidth_bottom + fieldwidth_bottom / 2;
                    p4.y = bottom_left.y;

                    run = true;
                }
                else if (type == 1 && i != FIELD_COUNT)
                {
                    p1.x = top_left.x;
                    p1.y = top_left.y + i * fieldheight_left + fieldheight_left / 2;
                    p2.x = top_right.x;
                    p2.y = top_right.y + i * fieldheight_right + fieldheight_right / 2;

                    p3.x = top_left.x + j * fieldwidth_top;
                    p3.y = top_left.y;
                    p4.x = bottom_left.x + j * fieldwidth_bottom ;
                    p4.y = bottom_left.y;

                    run = true;

                }
                if (run)
                {
                    line(img, p1, p2, Scalar(0, 0, 255), 1);
                    line(img, p3, p4, Scalar(0, 0, 255), 1);
                    Point roi_center;
                    intersection(p1, p2, p3, p4, roi_center);
                    //circle(img, roi_center, 10, Scalar(0, 0, 255), 2);

                    Rect r = Rect(roi_center.x - ROI_SIZE * (fieldwidth_top / 2),
                                  roi_center.y - ROI_SIZE * (fieldheight_left / 2),
                                  fieldwidth_top * ROI_SIZE,
                                  fieldheight_left * ROI_SIZE);

                    rectangle(img, r, Scalar(0, 0, 255), 2);

                    wall_t wall;
                    wall.exists = true;
                    wall.roi = r;
                    if (type == 0 && i != 8)
                    {
                        wall.pos = P_TOP;
                        wall.y = i;
                        wall.x = j;
                    }
                    if (type == 0 && i == 8)
                    {
                        wall.pos = P_BOTTOM;
                        wall.y = 7;
                        wall.x = j;
                    }
                    if (type == 1 && j != 8)
                    {
                        wall.pos = P_LEFT;
                        wall.x = j;
                        wall.y = i;
                    }
                    if (type == 1 && j == 8)
                    {
                        wall.pos = P_RIGHT;
                        wall.x = 7;
                        wall.y = i;
                    }
                    walls.push_back(wall);


                    if (debuglevel > 4)
                    {
                        cout << "i:" << i << " j:" << j << " type:" << type
                             << " Wall: " << pos_to_str[wall.pos]
                             << " [" << wall.x << "," << wall.y << "]\n";
                        rectangle(img, wall.roi, Scalar(0, 0, 255), 2);
                        laustracker_imshow("test", img);
                        cout << "press any key\n";
                        waitKey();
                    }

                }

            }
        }
    }

    if (debuglevel > 3) laustracker_imshow("cv_Grid", img);
    //imwrite("grid.jpg",img);
}

struct PositionCmp
{
    PositionCmp(const vector<Rect>& _positions) : positions(&_positions) {}
    bool operator()(int a, int b) const
    {
        int ax = (*positions)[a].x + (*positions)[a].width / 2;
        int ay = (*positions)[a].y + (*positions)[a].height / 2;
        int bx = (*positions)[b].x + (*positions)[b].width / 2;
        int by = (*positions)[b].y + (*positions)[b].height / 2;

        if (abs(ay - by) < (*positions)[a].height / 2)
        {
            ay = by;
        }


        if (ay < by) return true;
        if (ay == by) return ax < bx;
        return false; // (e1.first > 2.first )

    }
    const vector<Rect>* positions;
};

void getGrid2(const Mat& color_image, const Mat& white_image, const String mask_filename,
              const vector<Point> & labyrinth, vector <wall_t>& walls, int debuglevel)
{
    Mat mask_img = imread(mask_filename);
    if (mask_img.empty())
    {
        cout << "Error: " << mask_filename << " is no valid Mask image\n";
    }
    vector<Point> tmp_labyrinth = labyrinth;
    //scale_rect(tmp_labyrinth, 10);
    Rect roi = boundingRect(tmp_labyrinth);

    if (debuglevel > 2) cout << "Mask old size: " << mask_img.cols
                                 << "," << mask_img.rows << "\n";
    resize(mask_img, mask_img, Size(roi.width, roi.height));
    if (debuglevel > 2) cout << "Mask new size: " << mask_img.cols << ","
                                 << mask_img.rows << "\n";

    if (debuglevel > 3)
    {
        Mat color_img = color_image.clone();
        Mat imageROI = color_img(roi);
        addWeighted(imageROI, 0.5, mask_img, 0.5, 0., imageROI);
        laustracker_imshow_croped("cv_color_overlay", color_img, tmp_labyrinth);

        Mat white_img = white_image.clone();
        Mat imageROI2 = white_img(roi);
        addWeighted(imageROI2, 0.2, mask_img, 0.8, 0., imageROI2);
        drawLabyrinthRect(white_img, white_img, tmp_labyrinth);
        laustracker_imshow_croped("cv_white_overlay", white_img, tmp_labyrinth);
    }
    Mat contours_img = Mat::zeros(color_image.size(), CV_8UC3);
    Mat imageROI3 = contours_img(roi);
    addWeighted(imageROI3, 0., mask_img, 1, 0., imageROI3);
    if (debuglevel > 3) laustracker_imshow_croped("cv_mask_contours",
                contours_img, tmp_labyrinth);
    threshold(contours_img, contours_img, 128, 255, THRESH_BINARY);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat gray;
    cvtColor(contours_img, gray, CV_BGR2GRAY);

    findContours(gray, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    if (debuglevel > 3) cout << "\n\nHit any key to draw the next contour, "
                                 "ESC to quit\n\n";
    vector<int> sortIdx(contours.size());
    vector<Rect> positions(contours.size());
    for (int n = 0; n < (int)contours.size(); n++)
    {
        sortIdx[n] = n;
        positions[n] = boundingRect(contours[n]);
    }
    std::sort(sortIdx.begin(), sortIdx.end(), PositionCmp(positions));
    if (debuglevel > 3) laustracker_imshow_croped("cv_mask_contours",
                contours_img, tmp_labyrinth);

    Mat contours_tmp;
    for (int n = 0; n < (int)sortIdx.size(); n++)
    {
        int idx = sortIdx[n];

        if (debuglevel > 3)
        {
            contours_tmp = contours_img.clone();
            drawContours(contours_tmp, contours, idx,
                         Scalar(0, 0, 255), 2, 8, hierarchy,
                         0 // Try different values of max_level, and see what happens
                        );
            cout << "Contour #" << idx << ": position=" << positions[idx].x
                 << " " << positions[idx].y
                 << ", nvertices=" << contours[idx].size() << " n: " << n << "\n";
            laustracker_imshow_croped("cv_mask_contours", contours_tmp, tmp_labyrinth);
            // waitKey(100);
        }
        wall_t wall;
        wall.roi = boundingRect(contours[idx]);

        wall.exists = true;

        if (n > -1 && n < 8)
        {
            wall.pos = P_TOP; wall.y = 0; wall.x = n - 0;
        }
        else if (n >= 8 && n < 16)
        {
            wall.pos = P_LEFT; wall.y = 0; wall.x = n - 8;
        }
        else if (n == 16)
        {
            wall.pos = P_RIGHT; wall.y = 0; wall.x = 7;
        }

        else if (n > 16 && n <= 24)
        {
            wall.pos = P_TOP; wall.y = 1; wall.x = n - 17;
        }
        else if (n > 24 && n < 33)
        {
            wall.pos = P_LEFT; wall.y = 1; wall.x = n - 25;
        }
        else if (n == 33)
        {
            wall.pos = P_RIGHT; wall.y = 1; wall.x = 7;
        }

        else if (n > 33 && n <= 41)
        {
            wall.pos = P_TOP; wall.y = 2; wall.x = n - 34;
        }
        else if (n > 41 && n < 50)
        {
            wall.pos = P_LEFT; wall.y = 2; wall.x = n - 42;
        }
        else if (n == 50)
        {
            wall.pos = P_RIGHT; wall.y = 2; wall.x = 7;
        }

        else if (n > 50 && n <= 58)
        {
            wall.pos = P_TOP; wall.y = 3; wall.x = n - 51;
        }
        else if (n > 58 && n < 67)
        {
            wall.pos = P_LEFT; wall.y = 3; wall.x = n - 59;
        }
        else if (n == 67)
        {
            wall.pos = P_RIGHT; wall.y = 3; wall.x = 7;
        }

        else if (n > 67 && n <= 75)
        {
            wall.pos = P_TOP; wall.y = 4; wall.x = n - 68;
        }
        else if (n > 75 && n < 84)
        {
            wall.pos = P_LEFT; wall.y = 4; wall.x = n - 76;
        }
        else if (n == 84)
        {
            wall.pos = P_RIGHT; wall.y = 4; wall.x = 7;
        }

        else if (n > 84 && n <= 92)
        {
            wall.pos = P_TOP; wall.y = 5; wall.x = n - 85;
        }
        else if (n > 92 && n < 101)
        {
            wall.pos = P_LEFT; wall.y = 5; wall.x = n - 93;
        }
        else if (n == 101)
        {
            wall.pos = P_RIGHT; wall.y = 5; wall.x = 7;
        }

        else if (n > 101 && n <= 109)
        {
            wall.pos = P_TOP; wall.y = 6; wall.x = n - 102;
        }
        else if (n > 109 && n < 118)
        {
            wall.pos = P_LEFT; wall.y = 6; wall.x = n - 110;
        }
        else if (n == 118)
        {
            wall.pos = P_RIGHT; wall.y = 6; wall.x = 7;
        }

        else if (n > 118 && n <= 126)
        {
            wall.pos = P_TOP; wall.y = 7; wall.x = n - 119;
        }
        else if (n > 126 && n < 135)
        {
            wall.pos = P_LEFT; wall.y = 7; wall.x = n - 127;
        }
        else if (n == 135)
        {
            wall.pos = P_RIGHT; wall.y = 7; wall.x = 7;
        }
        else if (n > 135 && n <= 143)
        {
            wall.pos = P_BOTTOM; wall.y = 7; wall.x = n - 136;
        }
        else
        {
            cout << "undefined\n";
            wall.pos = P_BOTTOM;
            wall.x = 7;
            wall.y = 7;
            wall.exists = false;
        }

        walls.push_back(wall);
    }

    cout << "Finished all contours\n";

    //waitKey();
}

void check_for_wall(vector <wall_t>& walls, const Mat& white_img,
                    Mat& color_img, int debuglevel)
{

    //Mat mean_img=color_img.clone();
    for (size_t i = 0; i < walls.size(); i++)
    {
        Mat roi = white_img(walls[i].roi);


        int roisize = roi.rows * roi.cols;
        if (debuglevel > 2) cout << "ROI size     : " << roisize << "\n";

        int whitecount = countColor(roi, 255);
        if (debuglevel > 2) cout << "White Pixels : " << whitecount << "\n";

        int blackcount = countColor(roi, 0); //roisize - countNonZero(roi);
        if (debuglevel > 2) cout << "Black Pixels : " << blackcount << "\n";

        if (walls[i].exists)
        {
            if (whitecount > MIN_WHITE_PIXEL_PERCENTAGE / 100.0 * roisize)
            {
                rectangle(color_img, walls[i].roi, Scalar(0, 0, 255), 2);
                walls[i].exists = true;
            }
            else
            {
                rectangle(color_img, walls[i].roi, Scalar(255, 0, 0), 2);
                walls[i].exists = false;
            }
        }

        if (debuglevel > 2) cout << "check i:" << i << " Wall: "
                                     << pos_to_str[walls[i].pos] << " ["
                                     << walls[i].x << "," << walls[i].y << "] exists: "
                                     << walls[i].exists << "\n";

    }
    if (debuglevel > 2) laustracker_imshow("cv_dedect_walls_whitecount", color_img);

}

void check_for_black_line(vector <wall_t>& walls, const Mat& white_img,
                          Mat& color_img, int debuglevel)
{

    //Mat mean_img=color_img.clone();
    for (size_t i = 0; i < walls.size(); i++)
    {
        Mat roi = white_img(walls[i].roi);
        int roisize = roi.rows * roi.cols;
        if (debuglevel > 2) cout << "ROI size     : " << roisize << "\n";

        int whitecount = countColor(roi, 255);
        if (debuglevel > 2) cout << "White Pixels : " << whitecount << "\n";

        int blackcount = countColor(roi, 0); //roisize - countNonZero(roi);
        if (debuglevel > 2) cout << "Black Pixels : " << blackcount << "\n";

        if (walls[i].exists)
        {
            if ((whitecount > 0.1 * roisize) && (blackcount < 0.8 * roisize))
            {
                rectangle(color_img, walls[i].roi, Scalar(0, 0, 255), 2);
                walls[i].exists = true;
            }
            else
            {
                rectangle(color_img, walls[i].roi, Scalar(255, 0, 0), 2);
                walls[i].exists = false;
            }
        }

        if (debuglevel > 2) cout << "check i:" << i << " Wall: "
                                     << pos_to_str[walls[i].pos] << " ["
                                     << walls[i].x << "," << walls[i].y
                                     << "] exists: " << walls[i].exists << "\n";

    }
    if (debuglevel > 3) laustracker_imshow("cv_dedect_walls_whitecount", color_img);

}

