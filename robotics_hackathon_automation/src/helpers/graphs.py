from enum import Enum
from typing import Optional
import unittest
from .maze import Maze



class Direction(Enum):
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3
    
    def get_opposite(self):
        if self == Direction.UP:
            return Direction.DOWN
        if self == Direction.DOWN:
            return Direction.UP
        if self == Direction.LEFT:
            return Direction.RIGHT
        if self == Direction.RIGHT:
            return Direction.LEFT

def convert_maze(maze_array):
    # convert maze array to shapely polygon
    # [[0, 0, 0, 0, 1],
    #  [1, 1, 1, 1, 0],
    #  [0, 0, 0, 1, 0],
    #  [0, 1, 1, 1, 0],
    #  [0, 0, 0, 0, 0]]

    # to
    # [(1,0), (1,1), (1,2), (1,3), (1,4), (2,4), (3,4), (4,4), (4,3), (4,2), (4,1), (4,0), (3,0), (2,0), (1,0)]

    for j in range(len(maze_array)):
        state = 0
        start = 0
        end = 0
        for i in range(len(maze_array[j])):
            if maze_array[j][i] == 0:
                if state == 0:
                    state = 1
                    start = i
                    end = i
                else:
                    end = i
            else:
                if state == 1:
                    state = 0
                    if start != end:
                        yield [[start, j], [end, j]]
        if state == 1 and start != end:
            yield [[start, j], [end, j]]

    for i in range(len(maze_array[0])):
        state = 0
        start = 0
        end = 0
        for j in range(len(maze_array)):
            if maze_array[j][i] == 0:
                if state == 0:
                    state = 1
                    start = j
                    end = j
                else:
                    end = j
            else:
                if state == 1:
                    state = 0
                    if start != end:
                        yield [[i, start], [i, end]]
        if state == 1 and start != end:
            yield [[i, start], [i, end]]


class NodeGraph:
    def __init__(self, maze) -> None:
        self.nodes = []
        self.edges = []
        self.maze = maze
        self.node_dict = {}

        self.valid_points = [
            (j, i)
            for i in range(len(maze))
            for j in range(len(maze[0]))
            if maze[i][j] == 1
        ]

    @classmethod
    def from_maze(cls, maze, nodes, edges):
        graph = cls(maze)
        for node in nodes:
            graph.nodes.append(node)

        for edge in edges:
            graph.edges.append(edge)
        return graph

    def plot(self, node_color="red", edge_color="blue", ax=None):
        import matplotlib.pyplot as plt
        from matplotlib import axes
        if ax is None:

            _, ax = plt.subplots()

            # type hint for lsp
            ax: axes.Axes = ax
        # ax.pcolormesh(self.maze, cmap="gray")
        walls = convert_maze(self.maze)
        for wall in walls:
            ax.plot(*zip(*wall), color="black")
        for node in self.nodes:
            ax.scatter(*node, color=node_color)

        for edge in self.edges:
            ax.plot(*zip(*edge), color=edge_color)
        # for node in self.valid_points:
        #     ax.scatter(*node, color="blue")
        # plt.pcolormesh(maze)
        ax.set_aspect("equal")
        # plt.axes().set_aspect("equal")  # set the x and y axes to the same scale
        # plt.xticks([])
        # plt.yticks([])
        ax.invert_yaxis()
        plt.show()



    def get_max_edge(self, node, direction):
        """Get the length of the maximum possible edge from a node in a direction"""
        x, y = node
        if direction == Direction.UP:
            line = [arr[x] for arr in self.maze[:y]]
            try:
                wall = y - line[::-1].index(0)
            except ValueError:
                wall = 0
            return y - wall
        elif direction == Direction.DOWN:
            line = [arr[x] for arr in self.maze[y:]]
            try:
                wall = line.index(0)
            except ValueError:
                wall = len(line)
            return wall
        elif direction == Direction.LEFT:
            line = self.maze[y][:x]
            try:
                wall = x - line[::-1].index(0)
            except ValueError:
                wall = 0
            return x - wall
        elif direction == Direction.RIGHT:
            line = self.maze[y][x:]
            try:
                wall = line.index(0)
            except ValueError:
                wall = len(line)
            return wall

    def get_new_node(self, node, direction):
        if direction == Direction.UP:
            return node[0], node[1] - 1
        elif direction == Direction.DOWN:
            return node[0], node[1] + 1
        elif direction == Direction.LEFT:
            return node[0] - 1, node[1]
        elif direction == Direction.RIGHT:
            return node[0] + 1, node[1]

    def get_side_walls(self, node, from_direction):
        side_walls = {}
        for direction in Direction:
            if direction == from_direction or direction == from_direction.get_opposite():
                continue
            side_walls[direction] = self.get_max_edge(node, direction)
        return side_walls

    def add_node_edge(self, node1, node2):
            """Add edge between two nodes"""
            self.nodes.append(node2)
            self.edges.append((node1, node2))
            if self.node_dict.get(node1) is None:
                self.node_dict[node1] = [node2]
            else:
                self.node_dict[node1].append(node2)

            if self.node_dict.get(node2) is None:
                self.node_dict[node2] = [node1]
            else:
                self.node_dict[node2].append(node1)

    def explore_node(self, start_node, current_node ,from_direction: Direction = Direction.LEFT, \
                 side_walls: dict = {Direction.UP: 1, Direction.DOWN: 1, Direction.LEFT: 1, Direction.RIGHT: 1}):
        # All valid directions
        # print(start_node, current_node, from_direction, side_walls)
        for direction in Direction:
            # check if side walls are the same or becoming bigger
            if direction == from_direction or direction == from_direction.get_opposite():
                continue
            # If the max edge is greater than the side wall, then we can add an edge
            # print(self.get_max_edge(current_node, direction), side_walls[direction])
            if self.get_max_edge(current_node, direction) > side_walls[direction]:
                # print("adding edge")
                self.add_node_edge(start_node, current_node)
                new_node = self.get_new_node(current_node, direction)
                self.explore_node(current_node, new_node, direction.get_opposite(), self.get_side_walls(current_node, direction))
        # print(find_directions(self.maze, *current_node))
        if from_direction.get_opposite() not in find_directions(self.maze, *current_node):
            # print("end")
            if current_node != start_node:
                if current_node not in self.node_dict.get(start_node):
                    self.add_node_edge(start_node, current_node)
            return
        # print('exploring')
        try:
            self.explore_node(start_node, self.get_new_node(current_node, from_direction.get_opposite()), from_direction, self.get_side_walls(current_node, from_direction))
        except RecursionError:
            return (start_node, self.get_new_node(current_node, from_direction.get_opposite()), from_direction, self.get_side_walls(current_node, from_direction))


def add_edge(self, direction, node):
    """Add edge to the last node in the direction specified"""
    x, y = node
    # print("trying to add edge", direction, node)
    if direction == Direction.UP:
        # line = [arr[x] for arr in self.maze[:y]]
        # try:
        #     wall = y - line[::-1].index(0)
        # except ValueError:
        #     wall = 0

        node = [node for node in self.nodes if node[0] == x]
        # self.add_node_edge((x, wall), node)

        self.add_node_edge(max(node, key=lambda x: x[1]), (x, y))

    elif direction == Direction.LEFT:
        # line = self.maze[y][:x]
        # try:
        #     wall = x - line[::-1].index(0)
        # except ValueError:
        #     wall = 0
        # self.add_node_edge((wall, y), node)
        node = [node for node in self.nodes if node[1] == y]
        self.add_node_edge(max(node, key=lambda x: x[0]), (x, y))

    def get_edges(self, node):
        edges = []
        for edge in self.edges:
            if edges == 4:
                break
            if edge[0] == node:
                edges.append(edge)
            elif edge[1] == node:
                edges.append(edge)
        return edges



def find_directions(maze, x, y):
    directions = []
    if x > 0 and maze[y][x - 1] == 1:
        directions.append(Direction.LEFT)
    if (x < len(maze[y]) - 1) and maze[y][x + 1] == 1:
        directions.append(Direction.RIGHT)
    if y > 0 and maze[y - 1][x] == 1:
        directions.append(Direction.UP)
    if y < len(maze) - 1 and maze[y + 1][x] == 1:
        directions.append(Direction.DOWN)
    return directions


# def generate_nodes(maze_array) -> NodeGraph:
#     """Generate a graph of nodes and edges from a maze.
#
#     Args:
#         maze (Maze): The maze to generate the graph from.
#
#     Returns:
#         NodeGraph: The graph of nodes and edges.
#     """
#
#     graph = NodeGraph(maze_array)
#
#     for j in range(len(maze_array)):
#         for i in range(len(maze_array[j])):
#             if maze_array[j][i] == 1:
#                 directions = find_directions(maze_array, i, j)
#                 if len(directions) != 2 or set(directions) not in [
#                     {"up", "down"},
#                     {"left", "right"},
#                 ]:
#                     for direction in directions:
#                         if direction in ("up", "left"):
#                             graph.add_edge(direction, (i, j))
#
#                     graph.nodes.append((i, j))
#     return graph



def generate_nodes(maze_array, initial_nodes) -> NodeGraph:
    """Generate a graph of nodes and edges from a maze.
    Algorithm: Loop through initial nodes, recursively add edges to the nodes

    """

    graph = NodeGraph(maze_array)
    for node in initial_nodes:
        graph.nodes.append(node)
        graph.node_dict[node] = []
        for direction in find_directions(maze_array, *node):
            while True:
                a = graph.explore_node(node, graph.get_new_node(node, direction), direction.get_opposite(), graph.get_side_walls(node, direction))
                if a is None:
                    break
                else:
                    node, new_node, direction, side_walls = a
                    graph.explore_node(node, new_node, direction, side_walls)
    return graph


class TestGraph(unittest.TestCase):
    def test_graph(self):
        maze_array = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
            [0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0],
            [0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0],
            [0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0],
            [0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0],
            [0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
            [0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0],
            [0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0],
            [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1],
            [0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0],
            [0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ]

        class TestMaze(Maze):
            def __init__(self):
                self.entrance = (1, 1)
                self.exit = (19, 19)
                self.maze = maze_array

            def get_maze_array(self):
                return maze_array

        graph = generate_nodes(TestMaze())
        # graph.plot()
        self.assertEqual(len(graph.nodes), 73)
        self.assertEqual(len(graph.valid_points), 201)
        for cur, val in graph.node_dict.items():
            for node in val:
                print(node, cur, graph.node_dict[node])
                assert node in graph.nodes
                assert cur in graph.node_dict[node]


if __name__ == "__main__":
    TestGraph().test_graph()
    # maze = Maze(20, 20)
    # graph = generate_nodes(maze)
    # graph.plot()
