# This is necessary to find the main code`
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
        self.weights = self.weights = np.array([0 for x in range(2)])
        self.learning_rate = 0.2
        self.discount_factor = 0.8
        self.maxdepth = 1
        self.next_nodes = []
        self.visited = set()
        self.bomb_prob = 0
        self.bombs = set()


    def do(self, wrld):
        # Your code here
        #print (self.x, " ", self.y)
        if self.exit is None:
            x, y = self.find_exit(wrld)
        world = wrld.from_world(wrld)

        expectimaxresult = self.expectimax(wrld, self.x, self.y)
        if expectimaxresult[2]:
            self.place_bomb()
        self.move(expectimaxresult[0], expectimaxresult[1])


    def find_next_best(self, x, y, target, world, path):
        self.visited.add(target)
        this_path = self.pathfinding((x, y), target, world)

        if len(this_path) != 0:
            return (target, len(this_path))

        min_path = world.height()*world.width()
        min_point = (-1,-1)

        if self.next_nodes == []:
            surrounding = self.get_neighbors(target, world)
            for point in surrounding:
                length = len(self.visited)
                self.visited.add(point)
                if len(self.visited) > length:
                    self.next_nodes.append(point)

        for node in self.next_nodes:
            this_node = node
            if world.wall_at(node[0],node[1]):
                path += 1
            self.next_nodes.pop(0)
            found = self.find_next_best(x, y, this_node, world, path)

            if found[1] < min_path:
                min_path = found[1]
                min_point = found[0]



        return (min_point, min_path)



    def expectimax(self, wrld, x, y):

        print(self.find_exit(wrld))
        world = wrld.from_world(wrld)

        if (x,y)==self.find_exit(world):
            return (0,0, False)


        path = self.pathfinding((x, y), world.exitcell, wrld)
        neighbors = self.get_neighbors((x,y),wrld)

        #If unreachable
        if len(path) == 0:
            self.next_nodes.append(world.exitcell)
            self.visited.add(world.exitcell)
            next_best = self.find_next_best(x, y, self.next_nodes[0], world, world.height() * world.width())

            if next_best[0][0] == x and next_best[0][1] == y:
                for neighbor in neighbors:
                    if neighbor[0] - 1 == x and neighbor[1] - 1 == y:
                        return (neighbor[0]-x, neighbor[1]-y, True)
                    elif neighbor[0] + 1 == x and neighbor[1] + 1 == y:
                        return (neighbor[0]-x, neighbor[1]-y, True)
                    elif neighbor[0] - 1 == x and neighbor[1] + 1 == y:
                        return (neighbor[0]-x, neighbor[1]-y, True)
                    elif neighbor[0] + 1 == x and neighbor[1] - 1 == y:
                        return (neighbor[0]-x neighbor[1]-y, True)

            path = self.pathfinding((x,y), next_best[0], wrld)
            print(path)

        print(x, y)
        print(path)
        shortest_path = 0
        best_point = (x, y)
        for neighbor in neighbors:
            for point in path:
                if neighbor[0] == point[0] and neighbor[1] == point[1]:
                    print(neighbor[0])
                    print(point[0])
                    print(neighbor[1])
                    print(point[1])
                    if world.explosion_at(neighbor[0], neighbor[1]) or world.bomb_at(neighbor[0],
                                                                                     neighbor[1]):
                        return (0, 0, False)
                    return (neighbor[0]-x, neighbor[1]-y, False)


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
        # print(path)
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

    def find_bombs(self, wrld):
        """finds position of nearest bomb in world"""
        bombs = []

        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if wrld.bomb_at(x, y):
                    bombs.append((x, y))

        return bombs

    #These are our features I think
    def distance_to_monster(self, wrld, x, y):
        """check position of monster relative to character"""
        monsters = self.find_monsters(wrld) 

        if len(monsters) == 0:
            return 0 
        closest_m = monsters[0]
        
        for monster in monsters:
            d1 = self.get_distance((x,y), monster) 
            d2 = self.get_distance((x,y), closest_m)
            if d1 < d2:
                closest_m = monster 

        path = self.pathfinding((x,y), closest_m, wrld)
        length = len(path)
        if length == 0:
            return 0
        return 1/(length**2)

    # These are our features I think
    def distance_to_bombs(self, wrld, x, y):
        """check position of monster relative to character"""
        bombs = self.find_bombs(wrld)

        if len(bombs) == 0:
            return 0
        closest_b = bombs[0]

        for bomb in bombs:
            d1 = self.get_distance((x, y), bomb)
            d2 = self.get_distance((x, y), closest_b)
            if d1 < d2:
                closest_m = bomb

        path = self.pathfinding((x, y), closest_b, wrld)
        length = len(path)
        if length == 0:
            return 1
        return 1 / (length ** 2)

    def distance_to_exit(self, wrld, x, y):
        """check distance to exit"""

        exit_loc = self.find_exit(wrld) 
        path = self.pathfinding((x,y), exit_loc, wrld)
        length = len(path)
        if length == 0:
            return 1
        return 1/(length**2)

    def sum_rewards(self, wrld, x, y): 
        if wrld.exit_at(x, y):
            return wrld.time * 2 
        elif wrld.monsters_at(x,y) or wrld.explosion_at(x,y) or wrld.bomb_at(x,y):
            return -100
        return 1 

    def extract_features(self, wrld, x, y):
        f1 = self.distance_to_monster(wrld, x, y)
        f2 = self.distance_to_exit(wrld, x, y)
        #NEed to add more
        return [f1, f2]
    
    def q_value(self, wrld, x, y):
        fvec = self.extract_features(wrld, x, y)
        vals = list()
        i = 0
        for f in fvec:
            new_w = self.weights[i] * fvec[i]
            vals.append(new_w) 
            i += 1
        q = sum(vals)
        return q
    
    def next_best_q(self, wrld):
        neighbors = self.get_neighbors((self.x, self.y), wrld) 
        direction = [] 

        for n in neighbors:
            if wrld.empty_at(n[0], n[1]):
                direction.append((n[0] - self.x, n[1] - self.y)) 
        
        q_table = []
        for actions in direction:
            q_table.append(self.q_value(wrld, self.x, self.y))
        max_q = max(q_table) 
        return max_q

    def q_learn(self, wrld, x, y): 
        max_q = -999
        max_a = (0,0)

        neighbors = self.get_neighbors((self.x, self.y), wrld) 
        direction = [] 
        for n in neighbors:
            if wrld.empty_at(n[0], n[1]):
                direction.append((n[0] - self.x, n[1] - self.y)) 
        
        for move in direction:
            new_pos = x + move[0], y + move[1] 
            wrld.me(self).move(move[0], move[1])
            new_wrld, events = wrld.next()
            new_q = self.q_value(new_wrld, new_pos[0], new_pos[1])

            if new_q > max_q:
                max_q = new_q
                max_a = move

            return max_a

    def update_q(self, wrld, x, y): 
        fvec = self.extract_features(wrld, x, y)
        q = self.q_value(wrld, x, y)
        next_q = self.next_best_q(wrld)
        reward = self.sum_rewards(wrld, x, y)
        diff = reward + self.discount_factor * next_q - q
        i = 0 
        for f in fvec: 
            weight = self.weights[i] 
            updated_q = weight + self.learning_rate * diff * fvec[i]
            self.weights[i] = updated_q
            i += 1