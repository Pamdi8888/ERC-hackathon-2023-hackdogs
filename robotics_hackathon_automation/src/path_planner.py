#!/usr/bin/env python3
#Following is the import line for importing the obstacle detection methods i.e. isValidPoint()
#you need to name this node as "path_planner"

import rospy

from robotics_automation_hackathon.Coordinates import points
from helpers.graphs import NodeGraph
from matplotlib.patches import Rectangle
import obstacle_detection as obsdet

from helpers import generate_grid
from djikstra import djikstra_maze_array



maze = obsdet.maze
from matplotlib import pyplot as plt


# fig, [[ax1, ax2], [ax3, ax4]] = plt.subplots(2, 2)

# n = int(input("Enter the number of grids: "))
 # set limits of both axes to be the same
# ax1.set_xlim(-6, 6)
# ax1.set_ylim(-6, 6)
# ax2.set_xlim(-6, 6)
# ax2.set_ylim(-6, 6)
# ax3.set_xlim(0, n + 1)
# ax3.set_ylim(0, n + 1)


# import time

# a = time.perf_counter()
# print("Generate grid time: ", time.perf_counter() - a)
# poly = Polygon([(bounds[0], bounds[1]), (bounds[0], bounds[3]), (bounds[2], bounds[3]), (bounds[2], bounds[1])])
# ax.set_xlim(bounds[0] -1 , bounds[2]+ 1)
# ax.set_ylim(bounds[1] - 1, bounds[3] + 1)

# for wall in maze:
#     x, y = wall.polygon.exterior.xy
#     ax1.plot(x, y)
#
# for grid_box in level.grid:
#     x, y = grid_box.bounds[0], grid_box.bounds[1]
#     ax2.add_patch(Rectangle((x, y), level.square_width, level.square_width, fill=True, color='red'))

# Plot grid array 
# for i in range(len(level.grid_array)):
#     for j in range(len(level.grid_array[0])):
#         if level.grid_array[i][j] == 1:
#             ax3.add_patch(Rectangle((i * level.square_width, j * level.square_width), level.square_width, level.square_width, fill=True, color='red'))

# ax3.pcolormesh(level.grid_array, cmap='gray')


# ax.plot(poly.exterior.xy[0], poly.exterior.xy[1])

# print(level.get_grid_coord(-5.2, -2.19), level.get_grid_coord(3.10, 1.93))
# print(len(level.grid_array), len(level.grid_array[0]))
# print(level.grid_array[6][6], level.grid_array[41][75])
# plt.show()

rospy.init_node("path_planner")
publisher = rospy.Publisher('planned_path', points, queue_size=10)



nodes = []
edges = []
state = 1
for n in range(50, 100, 3):
    level = generate_grid(maze, n)
    start = (-5.2, -2.19)
    targets = [(5.18, -2.19), (1.58, -2.26),  (-2.28, 1.86), (0.57, 0.33), (-3.61, -2.20)]
#TODO find closest nodes and go there first

# ax.plot(poly.exterior.xy[0], poly.exterior.xy[1])

    print(len(level.grid_array), len(level.grid_array[0]))
# print(level.grid_array[6][6], level.grid_array[41][75])
# plt.show()

    nodes = []
    edges = []
    state = 1
    for i in targets:
        try:
            extra_nodes, extra_edges = djikstra_maze_array(level.grid_array, level.get_grid_coord(*start), level.get_grid_coord(*i))
        except:
            state = 0
            break
        nodes.extend(extra_nodes)
        edges.extend(extra_edges)
        start = i
    if state:
        break


if state:
    publisher.publish(nodes)
else:
    raise RuntimeError("Unable to generate path with n = 100")
        
# NodeGraph.from_maze(level.grid_array, nodes, edges).plot()
