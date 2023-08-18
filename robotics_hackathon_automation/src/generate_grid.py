from math import ceil, floor
from typing import List, Tuple
from matplotlib.patches import Rectangle

from shapely.geometry import Polygon
from obstacle_detection import Wall, maze


class Level:
    def __init__(self, maze: List[Wall], n: int, grid, bounds, square_width, grid_array) -> None:
        self.maze = maze
        self.n = n
        self.grid = grid
        self.bounds = bounds
        self.square_width = square_width
        self.grid_array = grid_array

    def get_grid_coord(self, x, y):
        i = floor((x - self.bounds[0]) / self.square_width)
        j = floor((y - self.bounds[1]) / self.square_width)
        return i, j


def maxi(a, b):
    return [ min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3]) ]

def generate_grid(maze: List[Wall], n) -> Level:
    # Divide the maze into n x n grid
    bounds: List[float] = list( maze[0].polygon.bounds)
    for wall in maze:
        bounds = maxi(bounds, wall.polygon.bounds)
    # maze.append(Wall(bounds[0], bounds[1], bounds[2], 0.15))
    bounds: List[float] = [ floor(bounds[0]), floor(bounds[1]), ceil(bounds[2]), ceil(bounds[3]) ]
    if bounds[3] - bounds[1] < bounds[2] - bounds[0]:
        width = bounds[2] - bounds[0]
        bounds[3] += (width - (bounds[3] - bounds[1])) 
    else:
        width = bounds[3] - bounds[1]
        bounds[2] += (width - (bounds[2] - bounds[0]))

    square_width = width / n
    grid = []
    grid_array = [list() for _ in range(n)]
    for i in range(n):
        for j in range(n):
            x = bounds[0] + i * square_width
            y = bounds[1] + j * square_width
            box = Polygon([(x, y), (x, y + square_width), (x + square_width, y + square_width), (x + square_width, y)])
            state = 0 
            for wall in maze:
                if box.intersects(wall.polygon):
                    state = 1
                    break
            if state == 1:
                grid.append(box)
                # 0 is wall
                grid_array[i].append(0)
            else:
                # 1 is path
                grid_array[i].append(1)
    return Level(maze, n, grid, bounds, square_width, grid_array)



    



if __name__ == "__main__":
    # generate_grid(maze, 10)
        

    from matplotlib import pyplot as plt


    fig, [ax1, ax2] = plt.subplots(1, 2)

     # set limits of both axes to be the same
    ax1.set_xlim(-6, 6)
    ax1.set_ylim(-6, 6)
    ax2.set_xlim(-6, 6)
    ax2.set_ylim(-6, 6)

    level = generate_grid(maze, 50)
# poly = Polygon([(bounds[0], bounds[1]), (bounds[0], bounds[3]), (bounds[2], bounds[3]), (bounds[2], bounds[1])])
# ax.set_xlim(bounds[0] -1 , bounds[2]+ 1)
# ax.set_ylim(bounds[1] - 1, bounds[3] + 1)

    display_maze = [Wall(-5.191, 0.9886, 1, 0.15), Wall(-5.639, -0.8309, 0.15, 3.769200), Wall(-5.672, 1.785, 0.15, 1.597130), Wall(-4.957, 2.543, 1.597130, 0.15), Wall(-4.277, 2.007956, 0.15, 1.169920), Wall(-0.0037, 2.51, 8.729630, 0.15), Wall(-1.588, 1.8136, 0.15, 1.25), Wall(-1.588, 0.0886, 0.15, 2.5), Wall(-2.138, 1.26, 1.25, 0.15), Wall(-2.668, 0.7136, 0.15, 1.25), Wall(-3.488, 0.16, 1.75, 0.15), Wall(2.405, 0.656, 0.75, 0.15), Wall(2.705, 0.956, 0.15, 0.75), Wall(3.2522, 1.2566, 1.25, 0.15), Wall(3.80526, 0.2066, 0.15, 2.25), Wall(3.3802, -0.844, 1, 0.15), Wall(2.955, -0.5433, 0.15, 0.75), Wall(2.7802, -0.2433, 0.5, 0.15), Wall(2.605, -0.5433, 0.15, 0.75), Wall(4.301, 2.189, 0.15, 0.810003), Wall(4.975, 2.5196, 1.50, 0.15), Wall(5.711, 1.998, 0.15, 1.192330), Wall(5.306, 1.463, 0.919672, 0.15), Wall(5.698, 0.301, 0.15, 2.276490), Wall(5.185, -0.885, 1.119670, 0.15), Wall(4.7, -1.296, 0.15, 0.982963), Wall(5.67, -1.7033, 0.15, 1.75), Wall(5.154, -2.521, 1.185380, 0.15), Wall(0.673, -2.534, 7.883080, 0.15), Wall(1.906, -1.93, 0.15, 1.206910), Wall(0.877, -1.7, 0.15, 1.719980), Wall(0.2502, -0.917, 1.50, 0.15), Wall(-0.433, -1.389, 0.15, 1.072), Wall(-0.4292, -0.4799, 0.15, 0.927565), Wall(0.9177, 0.2156, 0.15, 2.416050), Wall(0.23527, 1.3486, 1.5, 0.15), Wall(-0.439, 1.048, 0.15, 0.75), Wall(-3.2627, -1.72, 0.15, 1.75), Wall(-3.883, -0.9203, 1.414750, 0.15), Wall(-3.9377, -2.52, 1.5, 0.15), Wall(-4.615, -2.157, 0.15, 0.870384), Wall(2.105, 1.58, 0.15, 2.15893)]
    for wall in maze:
        x, y = wall.polygon.exterior.xy
        ax1.plot(x, y)

    for grid_box in level.grid:
        x, y = grid_box.bounds[0], grid_box.bounds[1]
        ax2.add_patch(Rectangle((x, y), level.square_width, level.square_width, fill=True, color='red'))

# ax.plot(poly.exterior.xy[0], poly.exterior.xy[1])


    plt.show()

