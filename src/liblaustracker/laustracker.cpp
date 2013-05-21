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

#include <laustracker/laustracker.h>
#include <laustracker/labyrinth.h>
#include <laustracker/util.h>
#include <laustracker/image_manipulation.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>

Mat img;
//the Window can be moved with the ALT-key
const char* locateLabyrinthWinName = "please_click_on_the_labyrinth_edges";
vector <Point> tmp_labyrinth_pos;
bool labyrinthSelected = false;

LausTracker::LausTracker()
{
    debuglevel = 0;
    labyrinth_found = false;
    undistort = true;
    wrap = true;
    mask_filename = "";
    use_mask = false;
    labyrinth_conf_filename = "../share/laustracker/labyrinth.conf";

    labyrinth_pos_path = string(getenv("HOME")) + "/.laustracker/labyrinth_pos.txt";
}

LausTracker::~LausTracker()
{
}

void locate_labyrinth_manually_on_mouse(int event, int x, int y, int /*flags*/, void*)
{
    if (img.empty())
        return;

    int updateFlag = 0;

    if (event == CV_EVENT_LBUTTONUP)
    {
        tmp_labyrinth_pos.push_back(Point(x, y));
        cout << "clicked: " << x << "," << y << "\n";

        updateFlag = true;
    }
    else if (event == CV_EVENT_RBUTTONUP)
    {
        tmp_labyrinth_pos.pop_back();
        cout << "deleted last point\n";
        updateFlag = true;
    }

    if (updateFlag)
    {
        Mat tmp_img = img.clone();
        //cout <<"labyrinth_pos.size(): "<<labyrinth_pos.size()<<"\n";
        for (int i = 0; i < tmp_labyrinth_pos.size(); i++)
        {
            circle(tmp_img, tmp_labyrinth_pos[i], 5, Scalar(0, 0, 255), 2);
        }
        if (tmp_labyrinth_pos.size() >= 2)
        {
            line(tmp_img, tmp_labyrinth_pos[0], tmp_labyrinth_pos[1],
                 Scalar(0, 0, 255), 2);
        }
        if (tmp_labyrinth_pos.size() >= 3)
        {
            line(tmp_img, tmp_labyrinth_pos[1], tmp_labyrinth_pos[2],
                 Scalar(0, 0, 255), 2);
        }
        if (tmp_labyrinth_pos.size() >= 4)
        {
            line(tmp_img, tmp_labyrinth_pos[2], tmp_labyrinth_pos[3],
                 Scalar(0, 0, 255), 2);
            line(tmp_img, tmp_labyrinth_pos[3], tmp_labyrinth_pos[0],
                 Scalar(0, 0, 255), 2);
            labyrinthSelected = true;
        }

        cv::imshow(locateLabyrinthWinName, tmp_img);


    }
}

bool LausTracker::locate_labyrinth_manually(const Mat& image)
{
    cout << "\n\n- The window [" << locateLabyrinthWinName
         << "] can be moved by holding down the Alt key\n"
         << "  and then just left-click dragging the window anywhere you’d like.\n"
         << "- Click on the labyrinth corners, clockwise rotation, start with the\n"
         << "  top left corner and select the outher most still blue area.\n"
         << "- You can undo your selections with the right mouse button.\n"
         << "- Hit the ESC key when you're ready\n\n";

    labyrinth_pos.clear();
    tmp_labyrinth_pos.clear();
    labyrinth_found = false;
    img = image.clone();
    Mat debug_img = image;
    cv::imshow(locateLabyrinthWinName, img);

    cvSetMouseCallback(locateLabyrinthWinName, locate_labyrinth_manually_on_mouse);

    while ((char)waitKey(100) != 27);

    destroyWindow(locateLabyrinthWinName);

    if (labyrinthSelected)
    {
        cout << "Manual search for the labyrinth was successful\n";
        drawLabyrinthRect(debug_img, debug_img, tmp_labyrinth_pos);
        labyrinth_pos = tmp_labyrinth_pos;

        // We need the outer border of the labyrinth to find walls
        scale_rect(labyrinth_pos, 30);

        robots.setLabyrinthPos(labyrinth_pos, img);

        if (wrap)
        {
            Wrapper.init(labyrinth_pos);
            labyrinth_pos = Wrapper.get_rect();
            Wrapper.wrap(image, img);
            if (debuglevel > 3) Wrapper.wrap(debug_img, debug_img);
        }
        if (debuglevel > 3)
        {
            drawLabyrinthRect(debug_img, debug_img, tmp_labyrinth_pos);
            drawLabyrinthRect(debug_img, debug_img, labyrinth_pos);
            laustracker_imshow_croped("Manual search", debug_img, labyrinth_pos);
        }
        labyrinth_found = true;
        return true;
    }
    else
    {
        cout << "Manual search for the labyrinth was _not_ successful\n";
        labyrinth_found = false;
        return false;
    }
}

bool LausTracker::locate_labyrinth_automatically(const Mat& image, bool reddots)
{
    labyrinth_pos.clear();
    labyrinth_found = false;
    vector<vector<Point> > squares;
    img = image;
    Mat debug_img;

    if (!reddots)
    {
        Mat contours_img;
        findSquares(img, squares, contours_img, debuglevel);
        findLabyrinth(contours_img, 80, labyrinth_pos, debuglevel);

        // We need the outer border of the labyrinth to find walls
        scale_rect(labyrinth_pos, 30);
    }
    else
    {
        findLabyrinth(img, labyrinth_pos, labyrinth_conf_filename, debuglevel);
    }

    if (debuglevel > 3)
    {
        debug_img = image.clone();
        squares.push_back(tmp_labyrinth_pos);
        drawSquares(debug_img, squares);
        //drawLabyrinthRect(debug_img, debug_img, labyrinth_pos);
        laustracker_imshow("debug_img", debug_img);
        cout << "Press any key\n";
        waitKey(0);
    }

    if (wrap)
    {
        Wrapper.init(labyrinth_pos);
        labyrinth_pos = Wrapper.get_rect();
        Wrapper.wrap(image, img);
        if (debuglevel > 3) Wrapper.wrap(debug_img, debug_img);
    }

    robots.setLabyrinthPos(labyrinth_pos, img);

    if (debuglevel > 3)
    {
        drawLabyrinthRect(debug_img, debug_img, tmp_labyrinth_pos);
        drawLabyrinthRect(debug_img, debug_img, labyrinth_pos);
        laustracker_imshow_croped("cv_automatic_search", debug_img, labyrinth_pos);
    }

    labyrinth_found = true;
    return true;
}

vector <Point> LausTracker::get_labyrinth_pos()
{
    return labyrinth_pos;
}

void LausTracker::locate_walls(Mat& image)
{
    if (labyrinth_found)
    {
        Mat white_img, blue_img;
#ifdef OLD_WHITE
        white_regions(image, white_img, 90);
#else
        /// Thresholds for the red markers at the labyrinth corners
        Scalar whitemin, whitemax;

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
        vector<int> gwmin =
            to_array<int>(labyrinth_conf.get<std::string>("walls.hsv_whitemin"));
        vector<int> gwmax =
            to_array<int>(labyrinth_conf.get<std::string>("walls.hsv_whitemax"));

        whitemin = Scalar(gwmin[0] * (179.0 / 360),
                          gwmin[1] * (255.0 / 100),
                          gwmin[2] * (255.0 / 100));
        whitemax = Scalar(gwmax[0] * (179.0 / 360),
                          gwmax[1] * (255.0 / 100),
                          gwmax[2] * (255.0 / 100));

        cvtColor(image, white_img, CV_BGR2HSV);
        inRange(white_img, whitemin, whitemax, white_img);

#define REMOVE_BLUE
#ifdef REMOVE_BLUE
        cvtColor(image, blue_img, CV_BGR2HSV);
        Scalar bluemin, bluemax;
        
        vector<int> gbmin =
            to_array<int>(labyrinth_conf.get<std::string>("walls.hsv_bluemin"));
        vector<int> gbmax =
            to_array<int>(labyrinth_conf.get<std::string>("walls.hsv_bluemax"));

        bluemin = Scalar(gbmin[0] * (179.0 / 360),
                          gbmin[1] * (255.0 / 100),
                          gbmin[2] * (255.0 / 100));
        bluemax = Scalar(gbmax[0] * (179.0 / 360),
                          gbmax[1] * (255.0 / 100),
                          gbmax[2] * (255.0 / 100));
        
        inRange(blue_img, bluemin, bluemax, blue_img);
        
        if (debuglevel > 3)laustracker_imshow("white_regins", white_img);
        if (debuglevel > 3)laustracker_imshow("blue_regins", blue_img);
        bitwise_not(blue_img, white_img, white_img);
#endif

#endif
        if (debuglevel > 3)laustracker_imshow("white_regins_for_walls", white_img);
        edm(white_img, white_img, 2, 3, 13);
        edm(white_img, white_img, 0, 5, 0);

        if (debuglevel > 3)laustracker_imshow("white_regins_for_walls_after_edm", white_img);
        cvtColor(white_img, white_img, CV_GRAY2BGR);

        vector <wall_t> walls;
        if (!use_mask)
        {
            tmp_labyrinth_pos = labyrinth_pos;
            scale_rect(tmp_labyrinth_pos, -10); // Get the border of the labyrinth
            getGrid(white_img, tmp_labyrinth_pos, walls, debuglevel);
        }
        else
        {
            getGrid2(image, white_img, mask_filename, labyrinth_pos, walls, debuglevel);
        }

        check_for_wall(walls, white_img, image, debuglevel);
        for (int i = 0; i < walls.size(); i++)
        {
            map.setWall(walls[i].x, walls[i].y, walls[i].pos, walls[i].exists);
            //cout<<"setWall i:"<<i<<" Wall: "<<pos_to_str[walls[i].pos]
            //    <<" ["<<walls[i].x<<","<<walls[i].y<<"] exists: "
            //    <<walls[i].exists<<"\n";
        }
    }
    else
    {
        cout << "Error: Labyrinth position unknown\n";
    }

}

void LausTracker::locate_black_lines(Mat& image)
{
    if (labyrinth_found)
    {
        Mat white_img, debug_img;

        white_regions(image, white_img, 70);
        //edm(white_img, white_img, 1, 3, 3);
        if (debuglevel > 3)
        {
            laustracker_imshow("white_regins_for_walls", white_img);

            vector<Point> tmp_labyrinth = labyrinth_pos;
            scale_rect(tmp_labyrinth, 10);
            drawLabyrinthRect(white_img, debug_img, tmp_labyrinth);
            laustracker_imshow("white_regins_for_walls_with_labyrinth_rect", debug_img);

            waitKey();
        }
        cvtColor(white_img, white_img, CV_GRAY2BGR);
        vector <wall_t> walls;
        getGrid2(image, white_img, mask_filename, labyrinth_pos, walls, debuglevel);
        if (debuglevel > 1) waitKey();
        check_for_black_line(walls, white_img, image, debuglevel);
        for (int i = 0; i < walls.size(); i++)
        {
            map.setWall(walls[i].x, walls[i].y, walls[i].pos, walls[i].exists);
            //cout<<"setWall i:"<<i<<" Wall: "<<pos_to_str[walls[i].pos]
            //    <<" ["<<walls[i].x<<","<<walls[i].y<<"] exists: "
            //    <<walls[i].exists<<"\n";
        }
    }
    else
    {
        cout << "Error: Labyrinth position unknown\n";
    }

}

void LausTracker::setDebug(int level)
{
    debuglevel = level;
    map.setDebug(level);
    robots.setDebug(level);
}

void LausTracker::update_robot_positions(Mat image)
{
    robots.processNewFrame(image);
}

bool LausTracker::labyrinthFound()
{
    return labyrinth_found;
}

void LausTracker::printMap()
{
#define MAP_SIZE_H 8*3+1
#define MAP_SIZE_V 8*2+1
    char  tmp_map[MAP_SIZE_H][MAP_SIZE_V];
    for (int i = 0; i < MAP_SIZE_V; i++)
        for (int j = 0; j < MAP_SIZE_H; j++)
            tmp_map[j][i] = '#';
    for (int i = 1; i < MAP_SIZE_V; i += 2)
        for (int j = 1; j < MAP_SIZE_H; j += 3)
        {
            tmp_map[j][i] = ' ';
            tmp_map[j + 1][i] = ' ';
        }


    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            if (!map.fields[j][i].wall_top)
            {
                //cout<<"no wall<<("<<j<<","<<i<<") tmp_map["
                //    <<3*j+1<<"]["<<(int)2*i<<"]\n";

                tmp_map[j * 3 + 1][2 * i] = ' ';
                tmp_map[j * 3 + 2][2 * i] = ' ';

            }
            if (!map.fields[j][i].wall_bottom)
            {

                tmp_map[j * 3 + 1][2 * i + 1] = ' ';
                tmp_map[j * 3 + 2][2 * i + 1] = ' ';

            }
            if (!map.fields[j][i].wall_left)
                tmp_map[j * 3][2 * i + 1] = ' ';
            if (!map.fields[j][i].wall_right)
                tmp_map[j * 3 + 3][2 * i + 1] = ' ';

            if (i > 0)
            {
                if (!map.fields[j][i].wall_top && !map.fields[j][i].wall_left
                        && !map.fields[j][i - 1].wall_left)
                    tmp_map[j * 3][2 * i] = ' ';

//                     if(j>0)
//                     {
//                         if(!fields[j-1][i].wall_top && !fields[j][i].wall_top
//                            && !fields[j][i-1].wall_left)
//                           tmp_map[j*3][2*i]=' ';
//                     }

            }
        }

    }

    if (robots.robots.size() > 0) //Wie haben Roboter in unser Liste
    {
        for (list<robot_t>::iterator robot = robots.robots.begin();
                robot != robots.robots.end(); ++robot)
        {
            if ((*robot).direction == P_TOP)
                tmp_map[(*robot).field_idx.x * 3 + 1]
                [2 * (*robot).field_idx.y + 1] = '^';
            if ((*robot).direction == P_BOTTOM)
                tmp_map[(*robot).field_idx.x * 3 + 1]
                [2 * (*robot).field_idx.y + 1] = 'v';
            if ((*robot).direction == P_LEFT)
                tmp_map[(*robot).field_idx.x * 3 + 1]
                [2 * (*robot).field_idx.y + 1] = '<';
            if ((*robot).direction == P_RIGHT)
                tmp_map[(*robot).field_idx.x * 3 + 1]
                [2 * (*robot).field_idx.y + 1] = '>';

        }
    }

    cout << "\n";

    cout << "============ Begin Map ===================\n";
    for (int i = 0; i < MAP_SIZE_V; i++)
    {
        for (int j = 0; j < MAP_SIZE_H; j++)
            cout << tmp_map[j][i];
        cout << "\n";
    }
    cout << "============ End Map =====================\n";

}

Mat LausTracker::improve_image(const Mat& image)
{
    Mat tmp = image;
    if (undistort)
        Undistort.remap(tmp, tmp);
    if (wrap && labyrinth_found)
    {
        Wrapper.wrap(tmp, tmp);
    }
    return tmp;
}

void LausTracker::imshow(const string& windoname, const Mat& image)
{
    if (labyrinth_found)
        laustracker_imshow_croped(windoname, image, labyrinth_pos);
    else
        laustracker_imshow(windoname, image);
}

void LausTracker::set_undistort(bool state)
{
    undistort = state;
}
void LausTracker::set_wrap(bool state)
{
    wrap = state;
}

void LausTracker::clear_labyrinth_pos()
{
    labyrinth_found = false;
    labyrinth_pos.clear();
}

void LausTracker::use_mask_to_locate_walls(String mask_img_filename)
{
    mask_filename = get_selfdir(debuglevel > 2) + mask_img_filename;
    cout << "mask_filename: " << mask_filename << "\n";
    if (mask_filename.empty())
    {
        use_mask = false;
    }
    else
    {
        use_mask = true;
    }
}

bool LausTracker::save_labyrinth_pos()
{
    boost::filesystem::path path(labyrinth_pos_path);
    boost::system::error_code returnedError;
    boost::filesystem::create_directories(path.parent_path(), returnedError);

    if (returnedError)
    {
        cout << "Error creating " << labyrinth_pos_path << " : " << returnedError << "\n";
        return false;
    }
    else
    {
        ofstream fileout(labyrinth_pos_path);
        for (int i = 0; i < 4; i++)
        {
            fileout << labyrinth_pos[i].x << " " << labyrinth_pos[i].y << " ";
        }

        return true;
    }
    return true;
}

bool LausTracker::load_labyrinth_pos()
{
    labyrinth_pos.clear();
    int x, y;
    ifstream filein(labyrinth_pos_path);
    for (int i = 0; i < 4; i++)
    {
        filein >> x; filein >> y;
        labyrinth_pos.push_back(Point(x, y));
    }
    robots.setLabyrinthPos(labyrinth_pos);
    labyrinth_found = true;
    return true;
}
