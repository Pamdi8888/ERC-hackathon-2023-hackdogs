#!/usr/bin/env python3
#Following is the import line for importing the obstacle detection methods i.e. isValidPoint()
#you need to name this node as "path_planner"

import rospy

from robotics_hackathon_automation.msg import Coordinates
from geometry_msgs.msg import Point
# from helpers.graphs import NodeGraph
# from matplotlib.patches import Rectangle
import obstacle_detection as obsdet
# from shapely.geometry import LineString

from helpers import generate_grid
from djikstra import djikstra_maze_array



from matplotlib import pyplot as plt


# fig, [[ax1, ax2], [ax3, ax4]] = plt.subplots(2, 2)

fig, ax = plt.subplots()
#
# # set limits of both axes to be the same
ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
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

rospy.init_node("path_planner")
publisher = rospy.Publisher('planned_path', Coordinates, queue_size=10)


maze = obsdet.maze
start = (-5.2, -2.19)
targets = [(5.18, -2.19), (1.58, -2.26),  (-2.28, 1.86), (0.57, 0.33), (-3.61, -2.20)]
for wall in maze:
    x, y = wall.polygon.exterior.xy
    ax.plot(x, y)

def find_closest(x, li):
    min_dist = (x[0] - li[0][0])**2 + (x[1] - li[0][1])**2
    closest = 0
    for el , i in enumerate(li):
        dist = (x[0] - i[0])**2 + (x[1] - i[1])**2
        if dist < min_dist:
            min_dist = dist
            closest = el
    return closest

i = 0
new_targets = [start]
while i < len(targets):
    index = find_closest(new_targets[-1], targets)
    new_targets.append(targets.pop(index))

nodes = []
edges = []
state = 1
for n in range(70, 100, 3):
    level = generate_grid(maze, n)

    # for grid_box in level.grid:
    #     x, y = grid_box.bounds[0], grid_box.bounds[1]
    #     ax2.add_patch(Rectangle((x, y), level.square_width, level.square_width, fill=True, color='red'))
#TODO find closest nodes and go there first

# ax.plot(poly.exterior.xy[0], poly.exterior.xy[1])


# print(level.grid_array[6][6], level.grid_array[41][75])
# plt.show()

    nodes = []
    edges = []
    state = 1
    for i in new_targets[1:]:
        try:
            extra_nodes, extra_edges = djikstra_maze_array(level.grid_array, level.get_grid_coord(*start), level.get_grid_coord(*i))
        except:
            state = 0
            break
        nodes.append(extra_nodes)
        print("n used", n)
        start = i
    if state:
        break

def convert( point):
    point = level.get_square_center(point[1], point[0])
    # return Point(x = point[0], y = point[1])
    return point


def check_useless(path):
    """
    Returns -1 if path is completely checked 
    Returns index if path stopped checking
    """
    valid = True
    for i, node in enumerate(path):
        if i == len(path ) - 1:
            return path
        valid = True
        for j, new_node in enumerate(path[:i+1:-1]):
            if obsdet.isValidPoint(*node, *new_node, maze):
                path = [node] + path[j:]
                valid = False

        if valid: 
            break
    # print(path)
    return path


    

new_nodes = []
for path in nodes:
    real_nodes = list(map(convert, path))
    new_nodes.append(real_nodes[::-1])
    for node in real_nodes[1:-1]:
        ax.scatter(*node, color='black')
    ax.scatter(*real_nodes[0], color='green')
    ax.scatter(*real_nodes[-1], color='red')
# plt.show()
# print(nodes)
# print(new_nodes, state)

total = []
i = 0
for path in new_nodes:
    total.append(new_targets[i])
    total.extend(path[1:-1])
    i+=1

total.append(new_targets[-1])

# print(total)

def makePoint(point):
    return Point(x = point[0], y = point[1])
if state:
    # i = 0
    # for node in total:
    #     i+=1
    #     ax.scatter(*node, alpha = i / len(total), color='black')
    # plt.show()
    publisher.publish(list(map( makePoint, total)))
else:
    raise RuntimeError("Unable to generate path with n = 100")
        
# NodeGraph.from_maze(level.grid_array, nodes, edges).plot()
