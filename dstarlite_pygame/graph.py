from utils import stateNameToCoords
from math import sqrt

class Node:
    def __init__(self, id):
        self.id = id
        # g approximation
        self.g = float('inf')
        # rhs value
        self.rhs = float('inf')
        self.neighbors = {}
        self.successors = {}
        self.inHeap = False;

class Graph:
    def __init__(self, x_dim, y_dim):
        self.x_dim = x_dim
        self.y_dim = y_dim
        # First make an element for each row (height of grid)
        self.cells = [0] * y_dim
        # Go through each element and replace with row (width of grid)
        for i in range(y_dim):
            self.cells[i] = [0] * x_dim
        self.graph = {}
        edge = 1
        for i in range(len(self.cells)):
            row = self.cells[i]
            for j in range(len(row)):
                # print('graph node ' + str(i) + ',' + str(j))
                node = Node('x' + str(i) + 'y' + str(j))
                dx = [1, 1, 0, -1, -1, -1,  0,  1]
                dy = [0, 1, 1,  1,  0, -1, -1, -1]
                for k in range(8):
                    newx = i + dx[k]
                    newy = j + dy[k]
                    if(newx >= 0 and newx < self.x_dim and newy >= 0 and newy < self.y_dim):
                        if(k%2 is 0):
                            node.neighbors['x' + str(newx) + 'y' + str(newy)] = edge
                        else:
                            node.neighbors['x' + str(newx) + 'y' + str(newy)] = edge*sqrt(2)
                node.successors = node.neighbors
                self.graph['x' + str(i) + 'y' + str(j)] = node


    def setStart(self, id):
        if(self.graph[id]):
            self.start = id
        else:
            raise ValueError('start id not in graph')

    def setGoal(self, id):
        if(self.graph[id]):
            self.goal = id
        else:
            raise ValueError('goal id not in graph')
