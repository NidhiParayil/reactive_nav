import numpy as np
from random import random, uniform
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
import sys
sys.path.append('/home/nidhi/reactive_nav/planner/')
from planners import Planner
from scripts import polygon



class Line():
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn


def Intersection(line, center, radius):
#   ''' Check line-sphere (circle) intersection '''
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a)
    t2 = (-b - np.sqrt(discriminant)) / (2 * a)

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True



def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))





def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False


def isThruObstacle(line, obstacles, radius):
    for obs in obstacles:
        if Intersection(line, obs, radius):
            return True
    return False


def nearest(G, vex, obstacles, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        # if isThruObstacle(line, obstacles, radius):
        #     continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx


def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
    return newvex



class Graph:
# ''' Define graph '''
    def __init__(self, startstate, endstate):
        self.startstate = startstate
        self.endstate = endstate

        self.vertices = [startstate]
        self.edges = []
        self.success = False

        self.vex2idx = {startstate:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}
        self.krrt_cost = {0:0.}

        self.sx = endstate[0] - startstate[0]
        # self.dsx = endstate[2] - startstate[2]
        self.sy = endstate[1] - startstate[1]
        # self.dsy = endstate[3] - startstate[3]

    def add_vex(self, state):
        try:
            idx = self.vex2idx[state]
        except:
            idx = len(self.vertices)
            self.vertices.append(state)
            self.vex2idx[state] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))


    def randomState(self):
        rx = random()
        ry = random()
        # drx = random()
        # dry = random()

        posx = self.startstate[0] - (self.sx / 2.) + rx * self.sx * 2
        posy = self.startstate[1] - (self.sy / 2.) + ry * self.sy * 2
        # velx = self.startstate[2] - (self.dsx / 2.) + drx * self.dsx * 2
        # vely = self.startstate[3] - (self.dsy / 2.) + dry * self.dsy * 2
        return posx, posy #, velx, vely

class RRT_star(Planner):
    def __init__(self):
        print("rrt star planner")
        self.n_iter= 100
        self.radius= 2
        self.stepSize=.7


    def def_cost(self, state):
        ob_sense_states =  [(.5, 2.0), (3.5,2), (3.5,3), (.5,3)]
        obstacle = polygon.MyPolygon(1,ob_sense_states)
        agent_states = [(state[0]-.5, state[1]-.5), (state[0]+.5, state[1]-.5), (state[0]+.5, state[1]+.5), (state[0]-.5, state[1]+.5)]
        # print("enter collision check", state[0], state[1]) 
        agent = polygon.MyPolygon(0,agent_states)
        agent_env = polygon.AgentPolygon(agent, [obstacle])
        
        # print(self.clearance(state) > 0.0, not(agent_env.check_poly_collision()))
        # agent_env.display()
        if agent_env.check_poly_collision() == True:
            # print("collided")
            return 0
        else:
            return 0

    def run_planner(self, cspace, ob_states=[]):
    #   ''' RRT star algorithm '''
        startstate = tuple(cspace.start_state[0:2])
        endstate = tuple(cspace.goal_state[0:2])
        G = Graph(startstate, endstate)
        Tree = []

        for _ in range(self.n_iter):
            randvex = G.randomState()
            # if isInObstacle(randvex, ob_states, self.radius):
            #     continue         

            nearvex, nearidx = nearest(G, randvex, ob_states, self.radius)
            if nearvex is None:
                continue

            newvex = newVertex(randvex, nearvex, self.stepSize)

            newidx = G.add_vex(newvex)

            c = distance([newvex[0],newvex[1]], [nearvex[0],nearvex[1]])
            # print("cost", c)
            c_def = self.def_cost(newvex)
            c =c +c_def
            # print("new cost", c)
            # print("collided", c_def)
            G.add_edge(newidx, nearidx, c)
            G.distances[newidx] = G.distances[nearidx] + c

            # update nearby vertices distance (if shorter)
            for vex in G.vertices:
                if vex == newvex:
                    continue

                    
                c = distance(vex, newvex)
                c_def = self.def_cost(vex)
                c =c +c_def
                if c > self.radius:
                    continue

                # line = Line(vex, newvex)
                # if isThruObstacle(line, obstacles, radius):
                #     continue

                idx = G.vex2idx[vex]
                    
                if G.distances[newidx] + c < G.distances[idx]:
                    G.add_edge(idx, newidx, c)
                    G.distances[idx] = G.distances[newidx] + c
            
                # acc_x, acc_y = get_trajectory(newvex, G.endstate)
                # c = cal_cost(acc_x, acc_y)
        
            c = distance(newvex, G.endstate)
            if c < 2 * self.radius:
                endidx = G.add_vex(G.endstate)
                G.add_edge(newidx, endidx, c)
                try:
                    # if krrt==True:
                    #     G.krrt_cost[endidx] = min(G.krrt_cost[endidx], G.krrt_cost[newidx]+c)
                    # else:
                    G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+c)
                except:
                    # if krrt==True:
                    #     G.krrt_cost[endidx] = G.krrt_cost[newidx]+c
                    # else:
                    G.distances[endidx] = G.distances[newidx]+c

                G.success = True
                # print('success')
                # break
                path = self.dijkstra(G)
                force = np.full(len(path), 0.1) 
        return path, force



    def dijkstra(self, G):
    #   '''
    #   Dijkstra algorithm for finding shortest path from start position to end.
    #   '''
        srcIdx = G.vex2idx[G.startstate]
        dstIdx = G.vex2idx[G.endstate]

        # build dijkstra
        nodes = list(G.neighbors.keys())
        dist = {node: float('inf') for node in nodes}
        prev = {node: None for node in nodes}
        dist[srcIdx] = 0
        # print(prev)
        while nodes:
            curNode = min(nodes, key=lambda node: dist[node])
            nodes.remove(curNode)
            if dist[curNode] == float('inf'):
                break

            for neighbor, cost in G.neighbors[curNode]:
                newCost = dist[curNode] + cost
                if newCost < dist[neighbor]:
                    dist[neighbor] = newCost
                    prev[neighbor] = curNode

        # retrieve path
        path = deque()
        curNode = dstIdx
        # print(prev)
        while prev[curNode] is not None:
            path.appendleft(G.vertices[curNode])
            curNode = prev[curNode]
        path.appendleft(G.vertices[curNode])
        return list(path)



if __name__ == '__main__':
    startstate = (0., 0.)
    endstate = (5., 5.)
    obstacles = []
    n_iter = 200
    radius = 0.5
    stepSize = 0.7

    G = RRT_star(startstate, endstate, obstacles, n_iter, radius, stepSize)
    # G = RRT(startpos, endpos, obstacles, n_iter, radius, stepSize)
    # print(G.neighbors)
    if G.success:
        path = dijkstra(G)
        print(path)
    # #     plot(G, obstacles, radius, path)
    # else:
    #     plot(G, obstacles, radius)

