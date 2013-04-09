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

#ifndef HEADER_LAUSTRACKER_CONF_H_INCLUDED
#define HEADER_LAUSTRACKER_CONF_H_INCLUDED

/**
 * @file conf.h
 * @brief Global definitions
 */

/// Minimum exposure time
#define UEYE_EXPOSURE_MIN 0.01

/// Maximum exposure time
#define UEYE_EXPOSURE_MAX 66.6

/// Minimum gain value
#define UEYE_HW_GAIN_MIN 0

/// Maximum gain value
#define UEYE_HW_GAIN_MAX 100

/// Hardware gain for master
#define UEYE_HW_GAIN_MASTER 50

/// Hardware gain for master (dark)
#define UEYE_HW_GAIN_MASTER_DARK 45

/// Hardware gain for red
#define UEYE_HW_GAIN_RED 12

/// Hardware gain for green
#define UEYE_HW_GAIN_GREEN 0

/// Hardware gain for blue
#define UEYE_HW_GAIN_BLUE 18

/// Normal exposure time in [ms]
#define UEYE_NORMAL_EXPOSURE_TIME 15.0

/// Dark exposure time to locate robots
#define UEYE_DARK_EXPOSURE_TIME 0.5

/// Minimum percentage the labyrinth takes from the the whole image
#define MIN_LABYRINTH_SIZE_PERCENT 40
/// Maximum percentage the labyrinth takes from the the whole image
#define MAX_LABYRINTH_SIZE_PERCENT 90

/// Number of labyrinth fields horizontally or vertically
#define FIELD_COUNT 8

/// Minimum percentage of the ROI that should be white to detect a wall
#define MIN_WHITE_PIXEL_PERCENTAGE 40

/// Size of the calculated ROI concerning field width
#define ROI_SIZE 0.3

#endif
