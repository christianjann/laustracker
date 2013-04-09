#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# This file is part of Laustracker.
#
# Copyright (C) 2013 Christian Jann <christian.jann@ymail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

'''
    Laustracker2D
    ~~~~~~~~~~~~~

    This script connects to the Laustracker server and displays
    a simple 2D map.

    :copyright: Copyright 2013 by Christian Jann
    :license: GPLv3+, see COPYING file
'''

from time import sleep
from optparse import OptionParser
import sys
import math
import pygame

import roslib
roslib.load_manifest('laustracker2d')
import rospy
from std_msgs.msg import String
from ltserver.msg import Map, Robots


MARKER_MAP_BEGIN = "START_MAP:"
MARKER_MAP_END = ":END_MAP"
MARKER_ROBOTS_BEGIN = "START_ROBOTS:"
MARKER_ROBOTS_END = ":END_ROBOTS"

FIELD_WIDTH_PX = 60
LAB_SIZE_PX = 8 * FIELD_WIDTH_PX + 9 + 1

FIELD_WIDTH_CM = 30.0
ROBOT_WIDTH_CM = 17.0
ROB_RADIUS = int(ROBOT_WIDTH_CM / FIELD_WIDTH_CM * FIELD_WIDTH_PX / 2.0)

NO_RELATIV_POS = False

TOP = 0
BOTTOM = 1
LEFT = 2
RIGHT = 3

screen = None
robots = []
fields = []
debug = False
show_robot_pos_cm = False


class Field:
    def __init__(self):
        self.wall_top = False
        self.wall_bottom = False
        self.wall_left = False
        self.wall_right = False
        self.buoy_top = False
        self.buoy_bottom = False
        self.buoy_left = False
        self.buoy_right = False


class Robot:
    def __init__(self):
        self.name = ""
        self.field_idx = (0, 0)
        self.direction = 0
        self.angle = 0
        self.relative_pos = (0, 0)
        self.third_led_found = False
        self.ratio = 0
        self.dist_blue_white = 0
        self.dist_red_white = 0
        self.absolut_pos_cm = (0, 0)


def print_map():
    MAP_SIZE_H = 8 * 3 + 1
    MAP_SIZE_V = 8 * 2 + 1
    # create a MAP_SIZE_V x MAP_SIZE_H matrix of '#'
    pmap = [['#' for col in range(MAP_SIZE_V)] for row in range(MAP_SIZE_H)]
    for i in range(1, MAP_SIZE_V - 1, 2):
        for j in range(1, MAP_SIZE_H - 1, 3):
            pmap[j][i] = ' '
            pmap[j + 1][i] = ' '
    for y in range(0, 8, 1):
        for x in range(0, 8, 1):
            if(fields[(x, y)].wall_top == False):
                pmap[x * 3 + 1][2 * y] = ' '
                pmap[x * 3 + 2][2 * y] = ' '
            if (fields[(x, y)].wall_bottom == False):
                pmap[x * 3 + 1][2 * y + 1] = ' '
                pmap[x * 3 + 2][2 * y + 1] = ' '
            if (fields[(x, y)].wall_left == False):
                pmap[x * 3][2 * y + 1] = ' '
            if (fields[(x, y)].wall_right == False):
                pmap[x * 3 + 3][2 * y + 1] = ' '
            if (y > 0):
                if (fields[(x, y)].wall_top == False
                    and fields[(x, y)].wall_left == False
                        and fields[(x, y - 1)].wall_left == False):
                    pmap[x * 3][2 * y] = ' '
    if(robots != None):
        for robot in robots:
            if(robot.direction == TOP):
                pmap[robot.field_idx[0] * 3 + 1]\
                    [2 * robot.field_idx[1] + 1] = '^'
            if(robot.direction == BOTTOM):
                pmap[robot.field_idx[0] * 3 + 1]\
                    [2 * robot.field_idx[1] + 1] = 'v'
            if(robot.direction == LEFT):
                pmap[robot.field_idx[0] * 3 + 1]\
                    [2 * robot.field_idx[1] + 1] = '<'
            if(robot.direction == RIGHT):
                pmap[robot.field_idx[0] * 3 + 1]\
                    [2 * robot.field_idx[1] + 1] = '>'
    map_str = ""
    for i in range(0, MAP_SIZE_V, 1):
        for j in range(0, MAP_SIZE_H, 1):
            map_str += pmap[j][i]
        map_str += "\n"
    rospy.loginfo("Map:\n%s\n" % map_str)


def chr2bool(string, idx):
    if (idx > -1 and idx < len(string)):
        if (string[idx] == '0'):
            return False
        else:
            return True
    else:
        print("wrong index")
        return False


def mapFromString(map_str):
    fields = {}
    for x in range(0, 8):
        for y in range(0, 8):
            fields[(x, y)] = Field()
    begin = map_str.find(MARKER_MAP_BEGIN)
    end = map_str.find(MARKER_MAP_END)
    if (begin > -1) and (end > -1):
        idx = begin + len(MARKER_MAP_BEGIN)
        for y in range(0, 8, 1):
            for x in range(0, 8, 1):
                idx += 1
                fields[(x, y)].buoy_top = chr2bool(map_str, idx)
                idx += 1
                fields[(x, y)].buoy_bottom = chr2bool(map_str, idx)
                idx += 1
                fields[(x, y)].buoy_left = chr2bool(map_str, idx)
                idx += 1
                fields[(x, y)].buoy_right = chr2bool(map_str, idx)
                idx += 1
                fields[(x, y)].wall_top = chr2bool(map_str, idx)
                idx += 1
                fields[(x, y)].wall_bottom = chr2bool(map_str, idx)
                idx += 1
                fields[(x, y)].wall_left = chr2bool(map_str, idx)
                idx += 1
                fields[(x, y)].wall_right = chr2bool(map_str, idx)
                idx += 1
                idx += 1
    else:
        print("begin or end missing, or both")
    return fields


def robotsFromString(robots_str):
    robots = []
    begin = robots_str.find(MARKER_ROBOTS_BEGIN)
    end = robots_str.find(MARKER_ROBOTS_END)
    robot_str = robots_str[begin + len(MARKER_ROBOTS_BEGIN) + 1:end - 1]
    for line in robot_str.splitlines():
        values = line.split()
        rob = Robot()
        rob.name = values[0]
        rob.field_idx = (int(values[1]), int(values[2]))
        rob.direction = int(values[3])
        rob.angle = float(values[4])
        rob.relative_pos = (float(values[5]), float(values[6]))
        rob.third_led_found = bool(values[7])
        rob.ratio = float(values[8])
        rob.dist_blue_white = float(values[9])
        rob.dist_red_white = float(values[10])
        rob.absolut_pos_cm = (float(values[11]), float(values[12]))
        robots.append(rob)
    if debug:
        print_robots(robots)
    return robots


def print_robots(robots):
    robots_str = "Robots:\n"
    for robot in robots:
        robots_str += " ".join([repr(el) for el in [robot.name, robot.field_idx,
                                                    robot.direction, robot.angle,
                                                    robot.absolut_pos_cm]])
        robots_str += "\n"
    rospy.loginfo("\n%s" % robots_str)


def wall(x, y, where):
    xc = FIELD_WIDTH_PX / 2 + 1 + (FIELD_WIDTH_PX + 1) * x
    yc = FIELD_WIDTH_PX / 2 + 1 + (FIELD_WIDTH_PX + 1) * y

    if(where == TOP):
        pygame.draw.line(
            screen, (255, 255, 255), (xc - FIELD_WIDTH_PX / 2,
                                      yc - FIELD_WIDTH_PX / 2),
            (xc + FIELD_WIDTH_PX / 2, yc - FIELD_WIDTH_PX / 2), 3)
    if(where == BOTTOM):
        pygame.draw.line(
            screen, (255, 255, 255), (xc - FIELD_WIDTH_PX / 2,
                                      yc + FIELD_WIDTH_PX / 2),
            (xc + FIELD_WIDTH_PX / 2, yc + FIELD_WIDTH_PX / 2), 3)
    if(where == LEFT):
        pygame.draw.line(
            screen, (255, 255, 255), (xc - FIELD_WIDTH_PX / 2,
                                      yc + FIELD_WIDTH_PX / 2),
            (xc - FIELD_WIDTH_PX / 2, yc - FIELD_WIDTH_PX / 2), 3)
    if(where == RIGHT):
        pygame.draw.line(
            screen, (255, 255, 255), (xc + FIELD_WIDTH_PX / 2,
                                      yc - FIELD_WIDTH_PX / 2),
            (xc + FIELD_WIDTH_PX / 2, yc + FIELD_WIDTH_PX / 2), 3)


def draw_bot(field_idx, angle, relative_pos=None):
    xc = int(FIELD_WIDTH_PX / 2 + 1 + (FIELD_WIDTH_PX + 1) * field_idx[0])
    yc = int(FIELD_WIDTH_PX / 2 + 1 + (FIELD_WIDTH_PX + 1) * field_idx[1])
    if relative_pos:
        xc = int(xc - relative_pos[0] * FIELD_WIDTH_PX)
        yc = int(yc - relative_pos[1] * FIELD_WIDTH_PX)
    pygame.draw.circle(screen, (0, 255, 0), (xc, yc), ROB_RADIUS, 4)
    dx = math.cos(math.radians(angle - 90)) * ROB_RADIUS
    dy = math.sin(math.radians(angle - 90)) * ROB_RADIUS
    pygame.draw.line(screen, (0, 255, 0), (xc, yc), (xc + dx, yc + dy), 4)


def update_display():
    screen.fill((0, 0, 150))
    for x in xrange(0, LAB_SIZE_PX, FIELD_WIDTH_PX + 1):
        pygame.draw.line(screen, (0, 0, 0), (x, 0), (x, LAB_SIZE_PX))
    for y in xrange(0, LAB_SIZE_PX, FIELD_WIDTH_PX + 1):
        pygame.draw.line(screen, (0, 0, 0), (0, y), (LAB_SIZE_PX, y))
    if fields:
        for y in range(0, 8, 1):
            for x in range(0, 8, 1):
                if(fields[(x, y)].wall_top == True):
                    wall(x, y, TOP)
                if(fields[(x, y)].wall_bottom == True):
                    wall(x, y, BOTTOM)
                if(fields[(x, y)].wall_right == True):
                    wall(x, y, RIGHT)
                if(fields[(x, y)].wall_left == True):
                    wall(x, y, LEFT)
    for robot in robots:
        if NO_RELATIV_POS:
            draw_bot(robot.field_idx, robot.angle)
        else:
            draw_bot(robot.field_idx, robot.angle, robot.relative_pos)

    if show_robot_pos_cm:
        if len(robots) != 0:
            pygame.display.set_caption(
                'Laustracker2D: Robot 1: ' +
                robots[0].name + ' [' +
                str(round(robots[0].absolut_pos_cm[0], 2)) + ',' +
                str(round(robots[0].absolut_pos_cm[1], 2)) + ']')

    pygame.display.update()


def handle_events():
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            rospy.signal_shutdown("pygame.quit()")
            sleep(1)
            return False
        elif event.type == pygame.QUIT:
            return False
    return True


def mapCallback(data):
    msg = str(data.data)
    if debug:
        rospy.loginfo(rospy.get_caller_id() + " I heard:\n%s\n", msg)
    if(msg.startswith(MARKER_MAP_BEGIN)):
        global fields
        fields = mapFromString(msg)
        if debug:
            print_map()
        update_display()


def mapMsgCallback(data):
    if debug:
        rospy.loginfo(rospy.get_caller_id() + " I heard:\n%s\n", data)
    global fields
    tmp_fields = {}
    for x in range(0, 8, 1):
        for y in range(0, 8, 1):
            tmp_fields[(x, y)] = Field()
            tmp_fields[(x, y)].buoy_top = data.map[x + y * 8].buoy_top
            tmp_fields[(x, y)].buoy_bottom = data.map[x + x * 8].buoy_bottom
            tmp_fields[(x, y)].buoy_left = data.map[x + y * 8].buoy_left
            tmp_fields[(x, y)].buoy_right = data.map[x + y * 8].buoy_right
            tmp_fields[(x, y)].wall_top = data.map[x + y * 8].wall_top
            tmp_fields[(x, y)].wall_bottom = data.map[x + y * 8].wall_bottom
            tmp_fields[(x, y)].wall_left = data.map[x + y * 8].wall_left
            tmp_fields[(x, y)].wall_right = data.map[x + y * 8].wall_right
    fields = tmp_fields
    if debug:
        print_map()
    update_display()


def robotsCallback(data):
    msg = str(data.data)
    if debug:
        rospy.loginfo(rospy.get_caller_id() + " I heard:\n%s\n", msg)
    if(msg.startswith(MARKER_ROBOTS_BEGIN)):
        global robots
        robots = robotsFromString(msg)
        update_display()


def robotsMsgCallback(data):
    if debug:
        rospy.loginfo(rospy.get_caller_id() + " I heard:\n%s\n", data)
    global robots
    tmp_robots = []
    for robot in data.robots:
        rob = Robot()
        rob.name = robot.name
        rob.field_idx = (robot.field_idx.x, robot.field_idx.y)
        rob.direction = robot.direction
        rob.angle = robot.angle
        rob.relative_pos = (robot.relative_pos.x, robot.relative_pos.y)
        rob.third_led_found = robot.third_led_found
        rob.ratio = robot.ratio
        rob.dist_blue_white = robot.dist_blue_white
        rob.dist_red_white = robot.dist_red_white
        rob.absolut_pos_cm = (robot.absolut_pos_cm.x, robot.absolut_pos_cm.y)
        tmp_robots.append(rob)
    robots = tmp_robots
    if debug:
        print_robots(robots)
    update_display()


if __name__ == "__main__":

    parser = OptionParser()
    parser.add_option("-d", "--debug",
                      action="store_true", dest="debug",
                      help="Print status messages to stdout")
    parser.add_option("-p", "--pos",
                      action="store_true", dest="pos",
                      help="Show robot position in window title")
    (options, args) = parser.parse_args()
    if options.debug:
        print("Running in debug mode")
        debug = True
    if options.pos:
        show_robot_pos_cm = True

    pygame.init()
    screen = pygame.display.set_mode((LAB_SIZE_PX, LAB_SIZE_PX), 0, 32)
    pygame.display.set_caption('Laustracker2D using pygame')
    update_display()

    rospy.init_node('laustracker2D', anonymous=True)

    # rospy.Subscriber("map_publisher", String, mapCallback)
    rospy.Subscriber("map_msg_publisher", Map, mapMsgCallback)
    # rospy.Subscriber("robots_publisher", String, robotsCallback)
    rospy.Subscriber("robots_msg_publisher", Robots, robotsMsgCallback)

    while handle_events() and not rospy.is_shutdown():
        rospy.sleep(0.5)

    pygame.quit()
