#!/usr/bin/env python3
#Following is the import line for importing the obstacle detection methods i.e. isValidPoint()
#you need to name this node as "path_planner"
from matplotlib.patches import Rectangle
import obstacle_detection as obsdet

from helpers import generate_grid
from djikstra import djikstra_maze_array



maze = obsdet.maze
from matplotlib import pyplot as plt


fig, [[ax1, ax2], [ax3, ax4]] = plt.subplots(2, 2)

n = int(input("Enter the number of grids: "))
 # set limits of both axes to be the same
ax1.set_xlim(-6, 6)
ax1.set_ylim(-6, 6)
ax2.set_xlim(-6, 6)
ax2.set_ylim(-6, 6)
ax3.set_xlim(0, n + 1)
ax3.set_ylim(0, n + 1)


import time

a = time.perf_counter()
level = generate_grid(maze, n)
print("Generate grid time: ", time.perf_counter() - a)
# poly = Polygon([(bounds[0], bounds[1]), (bounds[0], bounds[3]), (bounds[2], bounds[3]), (bounds[2], bounds[1])])
# ax.set_xlim(bounds[0] -1 , bounds[2]+ 1)
# ax.set_ylim(bounds[1] - 1, bounds[3] + 1)

for wall in maze:
    x, y = wall.polygon.exterior.xy
    ax1.plot(x, y)

for grid_box in level.grid:
    x, y = grid_box.bounds[0], grid_box.bounds[1]
    ax2.add_patch(Rectangle((x, y), level.square_width, level.square_width, fill=True, color='red'))

# Plot grid array 
# for i in range(len(level.grid_array)):
#     for j in range(len(level.grid_array[0])):
#         if level.grid_array[i][j] == 1:
#             ax3.add_patch(Rectangle((i * level.square_width, j * level.square_width), level.square_width, level.square_width, fill=True, color='red'))

ax3.pcolormesh(level.grid_array, cmap='gray')

# plt.show()



# ax.plot(poly.exterior.xy[0], poly.exterior.xy[1])

print(level.get_grid_coord(-5.2, -2.19), level.get_grid_coord(3.10, 1.93))
print(len(level.grid_array), len(level.grid_array[0]))
# print(level.grid_array[6][6], level.grid_array[41][75])
plt.show()

djikstra_maze_array(level.grid_array, level.get_grid_coord(-5.2, -2.19), level.get_grid_coord(3.10, 1.93), ax=ax3)
