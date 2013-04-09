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
    Script to visualize robot positions
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    This script reads robot positions from a csv file and
    visualizes them with matplotlib.

    :copyright: Copyright 2013 by Christian Jann
    :license: GPLv3+, see COPYING file
'''

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import MultipleLocator, AutoMinorLocator, FixedLocator

xs = []
ys = []
zs = []
rel_x = []
rel_y = []

with open('ltlog.csv', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in reader:
        # print ', '.join(row)
        print("Position: ", row[0], row[1])
        xs.append(float(row[0]))
        ys.append(float(row[1]))
        zs.append(0)
        rel_x.append(float(row[2]))
        rel_y.append(float(row[3]))


fig = plt.figure()
ax = fig.gca(projection='3d')

ax.plot(ys, xs, zs, label='Absolute Position')
ax.plot(ys, xs, rel_x, label='Relative X-Position')
ax.plot(ys, xs, rel_y, label='Relative Y-Position')
ax.set_xlabel("Y-Position in cm")
ax.set_ylabel("X-Position in cm")
ax.set_zlabel("Relative Position")
ax.legend()
ax.set_title('Absolute and Relative Robot Position')

# ax.w_xaxis.set_major_locator(FixedLocator([x for x in xrange(0,250,30)]))
# ax.w_yaxis.set_major_locator(FixedLocator([x for x in xrange(0,250,30)]))

ax.w_xaxis.set_major_locator(MultipleLocator(30))
ax.w_yaxis.set_major_locator(MultipleLocator(30))

ax.set_xlim3d(0, 240)
ax.set_ylim3d(0, 240)
ax.set_zlim3d(-2, 2)


fig = plt.figure(figsize=(6, 6), dpi=80)
ax = fig.add_subplot(111)
ax.plot(xs, ys, ',')
ax.grid(True)
ax.set_xlim(0, 240)
ax.set_ylim(0, 240)
ax.xaxis.set_major_locator(MultipleLocator(30))
ax.yaxis.set_major_locator(MultipleLocator(30))
ax.invert_yaxis()
ax.set_xlabel("X-Position in cm")
ax.set_ylabel("Y-Position in cm")
ax.set_title('Absolute Robot Position')

plt.show()
