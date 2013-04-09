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

#include <laustracker/map.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

const char* pos_to_str[] = {"TOP", "BOTTOM", "LEFT", "RIGHT", "CENTER", "OTHER"};

LabyrinthMap::LabyrinthMap()
{
    cout << "Map created\n";
    debuglevel = 0;
    test = false;
    init_map();

    map_path = string(getenv("HOME")) + "/.laustracker/map.txt";

}

LabyrinthMap::~LabyrinthMap()
{
    cout << "Destroyed Map\n";
}

void LabyrinthMap::init_map()
{
    field_t field_init;
    field_init.buoy_bottom = false;
    field_init.buoy_left = false;
    field_init.buoy_right = false;
    field_init.buoy_top = false;
    field_init.wall_bottom = false;
    field_init.wall_left = false;
    field_init.wall_right = false;
    field_init.wall_top = false;

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            fields[i][j] = field_init;
        }
    }
}

void LabyrinthMap::setDebug(int level)
{
    debuglevel = level;
}

void LabyrinthMap::set_single_wall(int x, int y, PosType pos, bool state)
{
    if (x >= 0 && y >= 0 && x <= 7 && y <= 7)
        switch (pos)
        {
        case P_TOP:
            fields[x][y].wall_top = state;
            break;
        case P_BOTTOM:
            fields[x][y].wall_bottom = state;
            break;
        case P_LEFT:
            fields[x][y].wall_left = state;
            break;
        case P_RIGHT:
            fields[x][y].wall_right = state;
            break;
        }
}

void LabyrinthMap::setWall(int x, int y, PosType pos, bool state)
{
    switch (pos)
    {
    case P_TOP:
        set_single_wall(x, y, P_TOP, state);
        set_single_wall(x, y - 1, P_BOTTOM, state);
        break;
    case P_BOTTOM:
        set_single_wall(x, y, P_BOTTOM, state);
        set_single_wall(x, y + 1, P_TOP, state);
        break;
    case P_LEFT:
        set_single_wall(x, y, P_LEFT, state);
        set_single_wall(x - 1, y, P_RIGHT, state);
        break;
    case P_RIGHT:
        set_single_wall(x, y, P_RIGHT, state);
        set_single_wall(x + 1, y, P_LEFT, state);
        break;
    }
}

string LabyrinthMap::toString()
{
    stringstream buf;
    buf << "START_MAP:";

    // Also update Python script and LabyrinthMap::fromString
    // if you change something
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            buf << "[";
            buf << fields[j][i].buoy_top << fields[j][i].buoy_bottom;
            buf << fields[j][i].buoy_left << fields[j][i].buoy_right;
            buf << fields[j][i].wall_top << fields[j][i].wall_bottom;
            buf << fields[j][i].wall_left << fields[j][i].wall_right;
            buf << "]";
        }
    }

    buf << ":END_MAP";
    return buf.str();
}

bool chr2bool(const string& str, int idx)
{
    if (idx > -1 && idx < str.size())
    {
        if (str[idx] == '0')
            return false;
        else
            return true;
    }
    else
        return false;
}

bool LabyrinthMap::fromString(string map_str)
{
    init_map();
    string marker_begin = "START_MAP:";
    string marker_end = ":END_MAP";
    int begin = map_str.find(marker_begin);
    int end = map_str.find(marker_end);

    if ((begin > -1) && (end > -1))
    {
        int idx = begin + marker_begin.length();
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 8; j++)
            {

                idx++; // buf<<"[";

                // buf<<fields[j][i].buoy_top<<fields[j][i].buoy_bottom;
                fields[j][i].buoy_top = chr2bool(map_str, idx++);
                fields[j][i].buoy_bottom = chr2bool(map_str, idx++);

                // buf<<fields[j][i].buoy_left<<fields[j][i].buoy_right;
                fields[j][i].buoy_left = chr2bool(map_str, idx++);
                fields[j][i].buoy_right = chr2bool(map_str, idx++);

                // buf<<fields[j][i].wall_top<<fields[j][i].wall_bottom;
                fields[j][i].wall_top = chr2bool(map_str, idx++);
                fields[j][i].wall_bottom = chr2bool(map_str, idx++);

                // buf<<fields[j][i].wall_left<<fields[j][i].wall_right;
                fields[j][i].wall_left = chr2bool(map_str, idx++);
                fields[j][i].wall_right = chr2bool(map_str, idx++);

                idx++; // buf<<"]";

            }
        }
    }
    else if (begin > -1)
    {
        cout << "only found begin\n";
        return false;
    }
    else if (end > -1)
    {
        cout << "only found end\n";
        return false;
    }
    else
    {
        cout << "no map found\n";
        return false;
    }

    return true;
}

bool LabyrinthMap::save()
{
    boost::filesystem::path path(map_path);
    boost::system::error_code returnedError;
    boost::filesystem::create_directories(path.parent_path(), returnedError);

    if (returnedError)
    {
        cout << "Error creating " << map_path << " : " << returnedError << "\n";
        return false;
    }
    else
    {
        ofstream fileout(map_path);
        fileout << toString();
        return true;
    }
    return true;
}

bool LabyrinthMap::load()
{
    ifstream filein(map_path);
    string map((std::istreambuf_iterator<char>(filein)),
               std::istreambuf_iterator<char>());
    fromString(map);
    return true;
}



