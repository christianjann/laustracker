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

// console application to undistort a image

#include <laustracker/laustracker.h>
#include <iostream>

using namespace cv;
using namespace std;

void help()
{
    cout << "\nusage:\n  ./undistort intrinsics.xml in.jpg out.jpg\n";
}

int main(int argc, char* argv[])
{

    if (argc != 4)
    {
        cout << "\nERROR: Wrong number of input parameters";
        help();
        return -1;
    }

    ImageUndistort Undistort;
    Undistort.set_intrinsics_xml(argv[1]);


    Mat image = imread(argv[2]), dst;
    if (!image.empty())
    {

        Undistort.remap(image, dst);
        laustracker_imshow("cv_original", image);
        laustracker_imshow("cv_undistorted", dst);
        imwrite(argv[3], dst);
        waitKey(0);
    }

    return 0;
}
