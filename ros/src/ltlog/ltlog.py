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
    Laustracker node to log robot positions
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    This script connects to the Laustracker server and
    writes robot positions to a csv file.

    :copyright: Copyright 2013 by Christian Jann
    :license: GPLv3+, see COPYING file
'''

import sys
import serial
import roslib
roslib.load_manifest('ltlog')
import rospy
from ltserver.msg import Map, Robots

# Change this to the name of your robot
robot_name = "Andi"

debug = False

f = file("ltlog.csv", "w")


def robotsMsgCallback(data):
    # Callback function that gets called when we receive a new
    # robots message from the Laustracker server
    if debug:
        rospy.loginfo(rospy.get_caller_id() + " I heard:\n%s\n", data)
    for robot in data.robots:
        if robot.name == robot_name:
            line = (str(robot.absolut_pos_cm.x) + ", " +
                    str(robot.absolut_pos_cm.y) + ", " +
                    str(robot.relative_pos.x) + ", " +
                    str(robot.relative_pos.y) + ", " +
                    str(robot.angle) + "\n")
            f.write(line)
            print(line)
            f.flush()

if __name__ == "__main__":

    rospy.init_node('ltxbee', anonymous=True)

    rospy.Subscriber("robots_msg_publisher", Robots, robotsMsgCallback)

    # endless loop
    while not rospy.is_shutdown():
        rospy.sleep(0.5)
