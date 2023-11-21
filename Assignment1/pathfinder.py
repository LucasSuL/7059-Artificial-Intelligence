import itertools
import math
import sys
import heapq


# any position (i, j) on the map.
class Node:
    def __init__(self, pos, parent=None, cost=0, order=None):
        self.pos = pos
        self.i = pos[0]  # row index (axis y)
        self.j = pos[1]  # column index (axis x)
        self.parent = parent
        self.cost = cost
        self.order = order

    # rewrite __lt__ for node direct comparision: list.sort()
    def __lt__(self, other):
        if self.cost == other.cost:
            return self.order < other.order
        return self.cost < other.cost


def solution(start, end, map, algorithm, heuristic):
    width = map[0].__len__()
    height = map.__len__()
    path = []
    node_list = [Node(start)]  # a list of Node object
    closed = set(start)  # an empty set -> "start" node automatically joined
    counter = itertools.count()  # track order

    # check if a position is valid on map (not out of boundary | not 'X' | not been checked)
    def isvalid(a_node):
        return (0 <= a_node.j < width and 0 <= a_node.i < height) and (map[a_node.i][a_node.j] != "X") and (
                a_node.pos not in closed)

    # if node.value - parent.value < 0 --> downhill --> 1, else: uphill--> 1 + ele_diff
    def one_move_cost(a_node):
        ele_diff = int(map[a_node.i][a_node.j]) - int(map[a_node.parent.i][a_node.parent.j])
        return 1 + max(0, ele_diff)

    # expand: up->down->left->right, index format as double array, return valid children
    def expand(a_node):
        actions = [(a_node.i - 1, a_node.j),
                   (a_node.i + 1, a_node.j),
                   (a_node.i, a_node.j - 1),
                   (a_node.i, a_node.j + 1)]

        # loop to get valid neighbor nodes
        nodelist_nbr = []  # empty list
        for i in range(actions.__len__()):
            node_nbr = Node(actions[i], a_node)  # create with parameter: 'parent'

            # available actions for positions adjacent to the map boundary or obstacles are reduced accordingly.
            if isvalid(node_nbr):
                nodelist_nbr.append(node_nbr)
        return nodelist_nbr

    if algorithm == "bfs":

        # loop to expand from start point
        while node_list:
            node = node_list.pop(0)

            # GOAL-TEST: when we reached the end point, loop to backtrack the path
            if node.pos == end:
                while node is not None:  # when a parent of this node has been tracked --> path
                    path.append(node)
                    node = node.parent
                return path

            # if not reach the end, expand node to get valid children
            for node_child in expand(node):
                node_list.append(node_child)
                closed.add(node_child.pos)  # track position

    # f(n) = g(n) = total cost to current node
    if algorithm == "ucs":

        while node_list:

            node_list.sort()  # always sort to pop the cheapest node
            node = node_list.pop(0)

            if node.pos == end:
                while node is not None:
                    path.append(node)
                    node = node.parent
                return path

            for node_child in expand(node):
                node_child.cost = one_move_cost(node_child) + node.cost  # cost = current move cost + previous cost
                node_child.order = next(counter)

                node_list.append(node_child)
                closed.add(node_child.pos)

    # f(n) = g(n) + h(n)
    if algorithm == "astar":

        def hn(pos):
            if heuristic == "manhattan":
                return abs(pos[0] - 9) + abs(pos[1] - 9)
            elif heuristic == "euclidean":
                return math.sqrt((9 - pos[0]) ** 2 + (9 - pos[1]) ** 2)

        node_list = [(18, 0, Node(start))]  # redefine node list: first element(fn = 18, order = 0, start point)
        heapq.heapify(node_list)

        while node_list:
            node = heapq.heappop(node_list)[2]

            if node.pos == end:
                while node is not None:
                    path.append(node)
                    node = node.parent
                return path

            for node_child in expand(node):
                cost = one_move_cost(node_child) + node.cost
                fn = cost + hn(node_child.pos)
                order = next(counter)

                node_new = Node(node_child.pos, node, cost, order)

                heapq.heappush(node_list, (fn, order, node_new))
                closed.add(node_child.pos)


# read input from Command Line Arguments
def read():
    input_file = sys.argv[1]
    algorithm = sys.argv[2]
    heuristic = None

    # handle heuristic if we are going to use A* search
    if len(sys.argv) > 3:
        heuristic = sys.argv[3]

    # extract lines form the given input file (sys.argv[1])
    file_ = open(input_file)
    lines = file_.readlines()
    file_.close()

    # first line -- the size of the map
    rows, columns = (int(lines[0].split(" ")[0]), int(lines[0].split(" ")[1]))
    # second line -- initial position
    start = (int(lines[1].split(" ")[0]) - 1, int(lines[1].split(" ")[1]) - 1)
    # third line -- destination position
    end = (int(lines[2].split(" ")[0]) - 1, int(lines[2].split(" ")[1]) - 1)
    # line 4 to line (4 + number of rows) -- map
    map = []
    for x in range(3, rows + 3):
        map.append(lines[x].strip().split())

    return start, end, map, algorithm, heuristic


def print_path(path):
    # create output file
    output_file = open("./init.txt", "w+")

    # superimpose stars on the map
    # case1: no path found
    if not path:
        print("null")
        output_file.write("null\n")

    # case2: modify format and write output file
    else:
        for node in path:
            map[node.i][node.j] = "*"
        for line in map:
            s = " ".join(line)
            print(s)
            output_file.write(s + "\n")
    output_file.close()


# main:
start, end, map, algorithm, heuristic = read()
path = solution(start, end, map, algorithm, heuristic)
print_path(path)
