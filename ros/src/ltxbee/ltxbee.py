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
    Laustracker XBee Node
    ~~~~~~~~~~~~~~~~~~~~~

    This script connects to the Laustracker server and sends
    the map and robot positions via XBee to the robots.

    :copyright: Copyright 2013 by Christian Jann
    :license: GPLv3+, see COPYING file
'''

from time import sleep
from optparse import OptionParser
import sys
import math
import serial

import roslib
roslib.load_manifest('ltxbee')
import rospy
from ltserver.msg import Map, Robots

from terminalcontroller import TerminalController
term = TerminalController()

TOP = 0
BOTTOM = 1
LEFT = 2
RIGHT = 3

robots = []
fields = []
debug = False
robot_name = None
serialport = None
newdata = False


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

    def disable(self):
        self.HEADER = ''
        self.OKBLUE = ''
        self.OKGREEN = ''
        self.WARNING = ''
        self.FAIL = ''
        self.ENDC = ''


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
        self.relativ_pos = (0, 0)
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


def str_direction(direction):
    if direction == TOP:
        return "TOP"
    elif direction == BOTTOM:
        return "BOTTOM"
    elif direction == LEFT:
        return "LEFT"
    elif direction == RIGHT:
        return "RIGHT"


def print_robots(robots):
    robots_str = "Robots:\n"
    for robot in robots:
        robots_str += " ".join([repr(el) for el in [robot.name, robot.field_idx,
                                                    str_direction(
                                                        robot.direction),
                                                    robot.angle,
                                                    robot.absolut_pos_cm]])
        robots_str += "\n"
    rospy.loginfo("\n%s" % robots_str)


def mapMsgCallback(data):
    if debug:
        rospy.loginfo(rospy.get_caller_id() + " I heard:\n%s\n", data)
    global fields, newdata
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
    newdata = True


def robotsMsgCallback(data):
    if debug:
        rospy.loginfo(rospy.get_caller_id() + " I heard:\n%s\n", data)
    global robots, newdata
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
    newdata = True


if __name__ == "__main__":

    print(term.render("${BLUE}Starting Laustracker XBee Node${NORMAL}"))

    port = "/dev/ttyUSB0"

    parser = OptionParser()
    parser.add_option("-d", "--debug",
                      action="store_true", dest="debug",
                      help="Print status messages to stdout")
    parser.add_option("-r", "--robot", dest="robot", default=None,
                      help="Specify a robot to track")
    parser.add_option("-p", "--port", dest="port", default=port,
                      help="Serial port to use")
    (options, args) = parser.parse_args()
    if options.debug:
        print("Running in debug mode")
        debug = True
    if options.robot:
        robot_name = options.robot
        print("Tracking robot: %s" % robot_name)
        port = "/dev/" + robot_name.lower() + "/xbee"

    if options.port:
        port = options.port

    try:
        print("Trying to open the serial port: " + port)
        serialport = serial.Serial(port, baudrate=57600, timeout=0.1)
        serialport.flushInput()
    except:
        print("Error opening serial port")
        serialport = None

    rospy.init_node('ltxbee', anonymous=True)

    rospy.Subscriber("map_msg_publisher", Map, mapMsgCallback)
    rospy.Subscriber("robots_msg_publisher", Robots, robotsMsgCallback)

    while not rospy.is_shutdown():
        myrobot = None
        if robots and newdata:
            if robot_name:
                for robot in robots:
                    if robot.name == robot_name:
                        myrobot = robot
            else:
                myrobot = robots[0]

            if myrobot != None:
                print_robots([myrobot])
                serialport.write("\n[" + str(int(myrobot.absolut_pos_cm[0])) + "," +
                                 str(int(myrobot.absolut_pos_cm[1])) + "] " +
                                 str(int(myrobot.angle)) + " " +
                                 str(int(myrobot.field_idx[0])) + " " +
                                 str(int(myrobot.field_idx[1])))
            newdata = False
        # rospy.sleep(0.5)

        try:
            tmp = serialport.read(1000)

            if tmp:
                # sys.stdout.write(tmp)
                rospy.loginfo(term.render("\n${GREEN}Received: " + tmp +
                                          "${NORMAL}"))

        except:
            print("Error reading from serial port")
