# This is necessary to find the main code
import math
import sys
import numpy as np
from queue import PriorityQueue
import node 

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back

class TestCharacter(CharacterEntity):
    def __init__(self, name, player, x, y):
        CharacterEntity.__init__(self, name, player, x, y) 
        self.exit = None 
        self.weights = np.array([0 for x in range(2)])
        self.learning_rate = 0.2
        self.discount_factor = 0.8

    def do(self, wrld):
        # Your code here
        # print (self.x, " ", self.y)
        if self.exit is None: 
            x, y = self.find_exit(wrld)
        path = self.pathfinding((self.x, self.y), (x,y), wrld) #hard code end point
        if (len(path) > 1):
            print(len(path))
            self.move(path[1][0] - self.x, path[1][1] - self.y)
        else:
            self.move(0,0)

    def pathfinding(self, start, end, world):  # start (x,y) and end (x,y)
        """Apply Astar to find the closest path from start to end in world"""
        startNode = self.create_node(start)
        endNode = self.create_node(end)
        visited = {}

        seenNeighbor = {}

        frontier = PriorityQueue()
        startNode.setCostSoFar(0)
        startNode.setEstCost(self.get_heuristic(start, end))
        frontier.put(startNode)
        visited[start] = 0

        seenNeighbor[start] = startNode

        while (not frontier.empty()):
            next = frontier.get()
            #visited[next.getNodePos()] = next

            if next.getNodePos() == endNode.getNodePos():
                return self.get_path(startNode, next)
            neighbor = self.get_neighbors(next.getNodePos(), world)
            for i in neighbor:
                # if wall, ignore
                if (world.wall_at(i[0], i[1])):
                    continue

                if not i in seenNeighbor:
                    node = self.create_node(i)
                    seenNeighbor[i] = node
                else:
                    node = seenNeighbor.get(i)


                if (next.getParent() == None or not (node == next.getParent())):
                # if the node is not next parents -> avoid duplicate path
                    if (next.getCostSoFar() == math.inf):
                        next.setCostSoFar(0)
                    cost = self.get_distance(next.getNodePos(), node.getNodePos()) + next.getCostSoFar()

                    if (cost < node.getCostSoFar()):
                        node.setParent(next)
                        node.setCostSoFar(cost)
                        node.setEstCost(cost + self.get_heuristic(node.getNodePos(), endNode.getNodePos()))
                        frontier.put(node)

        return [] #return empty if we find no path

    def get_heuristic(self, start, end):  # for now just compute distance
        """return the heuristic value from start point to end point"""
        return self.get_distance(start, end)  

    def get_distance(self, start, end):  # compute distance
        """return Manhatten distance from start to end point"""
        # return math.sqrt(math.pow(start[0] - end[0], 2) + math.pow(start[1] - end[1], 2))
        (x1,y1) = start[0], start[1]
        (x2,y2) = end[0], end[1]
        return abs(x1 - x2) + abs(y1 -y2)

    def get_path(self, startNode, endNode):
        """return the list of (x, y) point from start to end by backtracking parent node from end"""
        current = endNode
        path = []

        while (not current == startNode):
            path.insert(0, current.getNodePos())
            current = current.getParent()

        path.insert(0, startNode.getNodePos())
        print(path)
        return path


    def get_neighbors(self, pos, world):
        """get all the possible neighbor location and return a set of (x,y) neighbors"""
        width = world.width()
        height = world.height()
        neighbor = []

        if (pos[0] > 0):
            neighbor.append((pos[0] - 1, pos[1]))  # left
            if (pos[1] > 0):
                neighbor.append((pos[0] - 1, pos[1] - 1)) #diag left down
            if (pos[1] < height - 1):
                neighbor.append((pos[0] - 1, pos[1] + 1)) #diag left up

        if (pos[0] < width - 1):
            neighbor.append((pos[0] + 1, pos[1]))  # right
            if (pos[1] > 0):
                neighbor.append((pos[0] + 1, pos[1] - 1)) #diag right down
            if (pos[1] < height - 1):
                neighbor.append((pos[0] + 1, pos[1] + 1)) #diag right up

        if (pos[1] > 0):
            neighbor.append((pos[0], pos[1] - 1))  # down

        if (pos[1] < height - 1):
            neighbor.append((pos[0], pos[1] + 1))

        return neighbor

    def create_node(self, pos):
        """create a Node of a specific location"""
        return node.Node(pos)

    def find_exit(self, wrld): 
        """finds the position of the exit"""
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if wrld.exit_at(x,y):
                    return x,y

    def find_monsters(self,wrld):
        """finds position of nearest monster in world"""
        monsters = []

        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if wrld.monsters_at(x,y):
                    monsters.append((x,y))
        
        return monsters

    #These are our features I think
    def distance_to_monster(self, wrld, x, y):
        """check position of monster relative to character"""
        monsters = self.find_monsters(wrld) 

        if len(monsters) == 0:
            return 0 
        closest_m = monsters[0]
        
        for monster in monsters:
            d1 = get_distance((x,y), monster) 
            d2 = get_distance((x,y), )
            if get_distance((x,y), monster) < get_distance((x,y), closest_m):
                closest_m = monster 

        path = self.pathfinding(wrld, (x,y), closest_m)
        length = len(path)

        return 1/(length**2)

    def distance_to_exit(self, wrld, x, y):
        """check distance to exit"""

        exit_loc = self.find_exit(wrld) 

        if (x,y) == exit:
            return 1 
        else:
            path = self.pathfinding(wrld, (x,y), exit_loc)
            length = len(path) 

            return 1/(length**2) 

    def sum_rewards(self, wrld, x, y): 
        if wrld.exit_at(x, y):
            return wrld.time * 2 
        elif wrld.monster_at(x,y) or wrld.explosion_at(x,y) or wrld.bomb_at(x,y):
            return -100
        return 1 

    def extract_features(self, wrld, x, y):
        f1 = distance_to_monster(wrld, x, y)
        f2 = distance_to_exit(wrld, x, y)
        #NEed to add more
        return [f1, f2] 
    
    def q_value(self, wrld, x, y):
        fvec = self.extract_features(wrld, x, y)
        vals = list()
        for f in fvec:
            i = 0
            new_w = self.weights[i] * fvec[f]
            vals.append(new_w) 
            i += 1
        q = sum(vals)
        return q
    
    def next_best_q(self, wrld):
        neighbors = self.getNeighbor((self.x, self.y), wrld) 
        direction = [] 

        for n in neighbors:
            if wrld.empty_at(n[0], n[1]):
                direction.append((i[0] - self.x, i[1] - self.y)) 
        
        q_table = []
        for actions in direction:
            q_table.append(self.q_value(self.x, self.y))
        max_q = max(q_table) 
        return max_q

    def update_q(self, wrld, x, y): 
        fvec = self.extract_features(wrld, x, y)
        q = self.q_value(x, y)
        next_q = self.next_best_q(wrld)
        reward = self.sum_rewards(wrld, x, y)
        diff = reward + self.discount_factor * next_q - q
        
        for f in fvec: 
            i = 0 
            weight = self.weights[i] 
            updated_q = weight + self.learning_rate * diff * fvec[f]
            self.weights[i] = updated_q
        