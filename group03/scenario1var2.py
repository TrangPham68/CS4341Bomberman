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

class ExpectimaxCharacter(CharacterEntity):
    def __init__(self, name, player, x, y):
        CharacterEntity.__init__(self, name, player, x, y)
        self.exit = None
        self.weights = self.weights = np.array([0 for x in range(2)])
        self.learning_rate = 0.2
        self.discount_factor = 0.8
        self.maxdepth = 2
        self.next_nodes = []
        self.visited = set()
        self.bomb_prob = 0
        self.bombs = set()
        self.reviewed = set()


    def do(self, wrld):
        # Your code here
        #print (self.x, " ", self.y)
        if self.exit is None:
            x, y = self.find_exit(wrld)
        path = self.pathfinding((self.x, self.y), (x,y), wrld) #hard code end point
        #
        #if (len(path) > 1) and len(self.find_monsters(wrld)) == 0:
        #      self.move(path[1][0] - self.x, path[1][1] - self.y)
        # else:
        #     move = self.q_learn(wrld, self.x, self.y)
        #     self.update_q(wrld, self.x, self.y)
        self.visited = set()
        expectimaxresult = self.expectimax(wrld, self.x, self.y, 0, wrld.time)
        move = expectimaxresult[0]
        if self.bomb_prob > 500:
            print(self.bomb_prob)
            self.place_bomb()
        self.bomb_prob = self.bomb_prob*0.3
        print(expectimaxresult)
        self.move(int(move[0])-int(self.x), int(move[1])-int(self.y))


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


    def expectimax(self, wrld, x, y, depth, time):
        self.visited.add((x,y))

        (world, events) = wrld.next()

        #If at the exit
        if (x,y) == self.find_exit(world):
            self.bomb_prob = 0
            return ((x,y), 99999999)

        #If dead
        if world.explosion_at(x,y) or world.bomb_at(x,y):
            self.bomb_prob = 0
            return ((x, y), -99999999)

        #If dead
        monster_distance = self.distance_to_monster(world, x, y)
        if monster_distance == 0:
            return ((x,y), -9999999)

        #If dead
        character_distance = self.distance_to_character(world, x, y)
        if character_distance == 0:
            return ((x, y), -9999999)

        #Find path to exit
        path = self.pathfinding((x, y), world.exitcell, world)

        # Get the possible moves from starting position
        neighbors = self.get_neighbors((x, y), wrld)

        #List of next moves worth assessing
        free = set()
        for point in neighbors:
            if world.empty_at(x,y):
                free.add(point)

        #If unreachable, get the path to as close as you can
        if len(path) == 0:

            #Find the space that is as close as you can get
            self.next_nodes.append(world.exitcell)
            self.visited.add(world.exitcell)
            next_best = self.find_next_best(x, y, self.next_nodes[0], world, world.height() * world.width())

            #If you have reached that closest point, place a bomb and head to a diagonal neighbor
            if next_best[0][0] == x and next_best[0][1] == y:
                for neighbor in neighbors:
                    if world.characters_at(neighbor[0], neighbor[1]):
                        free.remove(neighbor[0], neighbor[1])
                        continue

                    #Place a bomb before you leave
                    self.bomb_prob = 1000.0
                    free = {}
                    free.add(neighbor[0] - 1, neighbor[1] - 1)
                    free.add(neighbor[0] + 1, neighbor[1] + 1)
                    free.add(neighbor[0] - 1, neighbor[1] + 1)
                    free.add(neighbor[0] + 1, neighbor[1] - 1)

            #Set the path to the close enough path
            path = self.pathfinding((x,y), next_best[0], world)

        #Attach monsters when possible
        if monster_distance == 1 or character_distance == 1:
            self.bomb_prob += 0.9
        elif monster_distance == 2 or character_distance == 1:
            self.bomb_prob += 0.6

        #Distance to closest bomb
        bomb_distance = self.distance_to_bombs(world,x,y)

        #In future bomb radius
        bomb_radius = set()
        bomb_danger = 0
        for bomb in self.find_bombs(world):
            bomb_radius.union(self.bomb_radius(bomb, world))
        if point in bomb_radius:
            if x == point[0] and y == point[1]:
                bomb_danger = 1.0

        #Calculate utility of current
        utility = 5/(len(path))**2
        utility -= 1/(monster_distance**3)
        utility -= 1 / (character_distance ** 3)
        utility -= (bomb_distance * (0.05) - bomb_danger*(0.5))

        #Calculate the basic utility of this choice
        if depth == self.maxdepth:
            return ((x, y), utility)

        max_pt = (-1,-1)
        max_val = -999999999

        for point in free:
            monster_pos = self.find_monsters(world)
            possible_monster = self.get_neighbors(monster_pos[0], world)
            freemon = []

            monster_val = 0
            monster_sum = 0
            monster_max = -99999999

            for point_2 in freemon:

                world.update_monster_move(world.monsters_at(x,y))
                world.update_character_move()

                (world2, events) = world.next()

                monster_exp = self.expectimax(world2, point[0], point[1], depth+1, time-1)
                self.bomb_prob = self.bomb_prob * 0.3

                if monster_exp[0][0] != -1 and monster_exp[0][1] != -1:
                    monster_val += monster_exp[1]

                if monster_val > monster_max:
                    monster_max = monster_val
                monster_sum += monster_val

            if monster_max + (monster_sum/len(freemon))*0.3 > max_val:
                max_pt = point
                max_val = monster_max + (monster_sum/len(freemon))*0.3


        return (max_pt, max_val/len(free) + utility)


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

    def find_characters(self,wrld):
        """finds position of nearest character in world"""
        characters = []

        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if wrld.characters_at(x,y):
                    characters.append((x,y))

        return characters

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


    def bomb_radius(self, bomb, world):
        bomb_radius = set()
        b_range = world.expl_range
        x_min = bomb[0]-b_range
        x_max = bomb[0]+b_range
        y_min = bomb[1] - b_range
        y_max = bomb[1] + b_range
        for x in  range(int(x_min), int(x_max)):
            if x == world.width():
                break
            if x < 0:
                x+=1
                continue
            if x == bomb[0]:
                bomb_radius.add((x,bomb[1]))
                x+=1
                continue
            if world.characters_at(x,bomb[1]) or world.bomb_at(x,bomb[1]) or world.wall_at(x,bomb[1]) or world.explosion_at(x,bomb[1]):
                bomb_radius.add((x,bomb[1]))
                break
            bomb_radius.add((x,bomb[1]))
            x+=1

        for y in range(y_min,y_max):
            if y == world.height():
                break
            if y < 0:
                y+=1
                continue
            if y == bomb[1]:
                bomb_radius.add((bomb[0],y))
                y+=1
                continue
            if world.characters_at(bomb[0],y) or world.bomb_at(bomb[0],y) or world.wall_at(bomb[0],y) or world.explosion_at(bomb[0],y) or x == world.width():
                bomb_radius.add((bomb[0],y))
                break
            bomb_radius.add((bomb[0],y))
            y+=1

        return bomb_radius