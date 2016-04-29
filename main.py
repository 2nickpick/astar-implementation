#
#   A-Star Simulation
#   Written by Nicholas Pickering
#   Artificial Intelligence
#   University of North Florida
#

import sys
from lib import util, astar
from shapely.geometry import Polygon, LineString

__author__ = 'Nicholas Pickering'

filename = 0

#   Start Main Program
print("A* Search Simulation")
print("Written by Nicholas Pickering")

#   Read in file for processing...
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    util.error("No filename specified... Exiting...", True)

file = open(filename, "r")
if not file:
    util.error("File could not be loaded... Exiting...", True)

#
#   Process File for Input Map
#   Load in Polygons and other states
#

start = None
goal = None
line_number = 0
for line in file.readlines():

    line = line.replace("\n", "").replace("\t", "").replace(" ", "").strip()

    # skip blank lines
    if len(line) <= 1:
        continue

    polygon = []
    polygon_points = line.split(';')
    for point in polygon_points:
        point = point.split(',')
        point = (int(point[0]), int(point[1]))

        if line_number == 0:
            start = point
        elif line_number == 1:
            goal = point
        else:
            polygon.append(point)

    if len(polygon) > 0:
        astar.polygons.append(Polygon(polygon))
        polygon = []

    line_number += 1

#
# Begin the A-Star Search Algorithm
#

print()
# print("Current State: " + str(start))

results = astar.run(start, goal)

if not results:
    print("No solution was found for this data set.")
else:
    print("The optimal path is " + str(results))

