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

#ifndef HEADER_LAUSTRACKER_MAP_H_INCLUDED
#define HEADER_LAUSTRACKER_MAP_H_INCLUDED

/**
 * @file map.h
 * @brief The class LabyrinthMap contains complete information about the labyrinth
 *        structure.
 */

#include <iostream>
using namespace std;

/**
 * @brief Possible positions within a field.
 *
 */
typedef enum PosEnum { P_TOP, P_BOTTOM, P_LEFT, P_RIGHT, P_CENTER, P_OTHER } PosType;
extern const char* pos_to_str[];

/**
 * @brief Possible states of buoys.
 *
 */
enum { red, green, neutral, none };

/**
 * @brief Data structure for a single labyrinth field.
 *
 * There are more efficient ways to save the labyrinth map than a 8x8 array of
 * type field_t but this should be easy to debug and extend.
 */
typedef struct field_t
{
    bool wall_top, wall_bottom, wall_left, wall_right;
    bool buoy_top, buoy_bottom, buoy_left, buoy_right;

} field_t;


/**
 * @brief Class contains complete information about the labyrinth
 *        structure.
 *
 */
class LabyrinthMap
{

private:

    void set_single_wall(int x, int y, PosType pos,  bool state);
    int debuglevel;
    bool test;
    void init_map();

public:

    /// Contains data about every field.
    field_t fields[8][8];

    /// File to save or load the map
    string map_path;

    /**
     * \brief Constructor.
     *
     * Initialization of the map.
     *
     */
    LabyrinthMap();

    /**
     * \brief Deconstructor
     *
     * Free memory.
     *
     */
    ~LabyrinthMap();

    /**
     * \brief Set or remove a wall.
     *
     * \param x x-coordinate
     * \param y y-coordinate
     * \param pos Position within a field (top, bottom, left, right, center, other)
     * \param state true: wall exists false: wall does not exist
     *
     */
    void setWall(int x, int y, PosType pos, bool state);

    /**
     * \brief Check if the wall exists.
     *
     * \param x x-coordinate
     * \param y y-coordinate
     * \param pos Position within a field (top, bottom, left, right, center, other)
     *
     * \return true or false, true if the wall exists
     *
     */
    bool getWall(int x, int y, PosType pos);

    void setDebug(int level);

    /**
     * \brief Return the serialized map as string.
     * \return Serialized map.
     *
     */
    string toString();

    /**
     * \brief Read the map from a string.
     *
     * \param map_str String that contains a map.
     *
     * \return true, if a map was found.
     *
     */
    bool fromString(string map_str);

    /**
     * \brief Save the map to ~/.laustracker/map.txt.
     *
     * \return true, if the map could be saved.
     *
     */
    bool save();

    /**
     * \brief Load the map from ~/.laustracker/map.txt.
     *
     * \return true, if the map was loaded.
     *
     */
    bool load();

};

#endif
