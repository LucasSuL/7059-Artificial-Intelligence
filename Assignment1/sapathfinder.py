import copy
import math
import random
import sys
import pathfinder


class Node:
    def __init__(self, pos, parent=None):
        self.pos = pos
        self.i = pos[0]  # row index (axis y)
        self.j = pos[1]  # column index (axis x)
        self.parent = parent


def rand_bfs(start, end, map):
    width = map[0].__len__()
    height = map.__len__()
    path = []
    node_list = [Node(start)]  # a list of Node object
    closed = set(start)  # an empty set -> "start" node automatically joined

    # check if a position is valid on map (not out of boundary | not 'X' | not been checked)
    def isvalid(a_node):
        return (0 <= a_node.j < width and 0 <= a_node.i < height) and (map[a_node.i][a_node.j] != "X") and (
                a_node.pos not in closed)

    def expand(a_node):
        actions = [(a_node.i - 1, a_node.j),
                   (a_node.i + 1, a_node.j),
                   (a_node.i, a_node.j - 1),
                   (a_node.i, a_node.j + 1)]

        # loop to get valid neighbor nodes
        nodelist_nbr = []  # empty list

        # randomize action list
        random.shuffle(actions)

        for i in range(actions.__len__()):
            node_nbr = Node(actions[i], a_node)  # create with parameter: 'parent'

            # available actions for positions adjacent to the map boundary or obstacles are reduced accordingly.
            if isvalid(node_nbr):
                nodelist_nbr.append(node_nbr)
        return nodelist_nbr

    # def one_move_cost(a_node):
    #     ele_diff = int(map[a_node.i][a_node.j]) - int(map[a_node.parent.i][a_node.parent.j])
    #     return 1 + max(0, ele_diff)

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


def rand_local_adjust(pos_list, d):
    # d = d
    # get random (u,v)
    index = random.randint(0, pos_list.__len__() - 1)
    u, v = pos_list[index]

    # pick (x,y)
    if index + d > pos_list.__len__() - 1:
        random_end_index = pos_list.__len__() - 1
    else:
        random_end_index = index + d
    x, y = pos_list[random_end_index]

    node_list_random_part = rand_bfs((u, v), (x, y), map)  # len = 6
    path_random_part = [item.pos for item in node_list_random_part]
    path_new = pos_list.copy()

    for i in range(index, random_end_index):
        path_new[i] = path_random_part[i - index] #??
    return path_new


def get_cost(path):
    cost = 0
    for i in range(1, len(path)):
        x1, y1 = path[i - 1]
        x2, y2 = path[i]
        cost += 1 + max(int(map[x2][y2]) - int(map[x1][y1]), 0)
    return cost


def annealing(t_ini, t_fin, alpha, path, d):  # 3 args + initial path
    log = []
    while t_ini > t_fin:
        path_new = rand_local_adjust(path, d)
        g_ini = get_cost(path)
        g_fin = get_cost(path_new)
        delta_g = g_ini - g_fin

        # if better then substitute former path
        if delta_g > 0:
            path = path_new
        else:
            chance = math.exp(float(delta_g) / float(t_ini))  # e pow ()
            if random.random() < chance:
                path = path_new
        log.append((t_ini, g_ini))

        t_ini *= alpha
    return path, log


# read input from Command Line Arguments
def read():
    input_file = sys.argv[1]
    path_input = sys.argv[2]
    t_ini = int(sys.argv[3])
    t_fin = float(sys.argv[4])
    alpha = float(sys.argv[5])
    d = int(sys.argv[6])

    # extract lines form the given input file (sys.argv[1])
    file = open(input_file)
    path_file = open(path_input)

    lines = file.readlines()
    file.close()

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

    original_solution = []  # = map
    for line in path_file.readlines():
        line = line.strip().split()
        original_solution.append(line)

    pos_list = []  # [(a,b),(c,d)...]
    for i in range(len(original_solution)):
        for j in range(len(original_solution[0])):
            if original_solution[i][j] == "*":
                pos_list.append((i, j))

    path_file.close()
    return start, end, map, pos_list, t_ini, t_fin, alpha, d


# main:
start, end, map, pos_list, t_ini, t_fin, alpha, d = read()

annealing_path, log = annealing(t_ini, t_fin, alpha, pos_list, d)
for x, y in annealing_path:
    map[x][y] = "*"

for line in map:
    print(" ".join(line))

for t, cost in log:
    print("T = " + "{:.6f}".format(t) + ", cost = " + str(int(cost)))
