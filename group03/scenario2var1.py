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

class AStarCharacter(CharacterEntity):
    def __init__(self, name, player, x, y):
        CharacterEntity.__init__(self, name, player, x, y)
        self.next_nodes = []
        self.visited = set()



    def do(self, wrld):
        maxresult = self.get_best(wrld, self.x, self.y)
        if maxresult[2]:
            self.place_bomb()
        print(maxresult)
        self.move(maxresult[0], maxresult[1])

    def closest_accessable_character(self, wrld, x, y):
        """return the closest character"""

        chars = self.find_characters(wrld)

        if len(chars) == 0:
            return 0

        closest_c = (-1,-1)
        closest_c_path = []

        check_neighbors = set()
        for char in chars:
            path_to_char = self.pathfinding((x, y), char, wrld)
            if char[0] == x and char[1] == y:
                continue
            elif len(path_to_char) == 0:
                check_neighbors.append(char)
            elif len(closest_c_path) == 0:
                closest_c = char
                closest_c_path = self.pathfinding((x, y), char, wrld)
            elif len(path_to_char) < len(closest_c_path):
                closest_c = char
                closest_c_path = self.pathfinding((x, y), char, wrld)

        return closest_c

    def find_characters(self,wrld):
        """finds position of nearest character in world"""
        characters = []

        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if wrld.characters_at(x,y):
                    characters.append((x,y))

        return characters

    def get_best(self, wrld, x, y):
        """Return the best possible move and whether to place a bomb"""

        #Create a sensed world
        world = wrld.from_world(wrld)

        #Find the closest opponent and find the spaces 2 away
        closest_char = self.closest_accessable_character(wrld,x,y)

        all_bombs = self.find_bombs(wrld)
        blast_radius = set()
        for bomb in all_bombs:
            radius = self.bomb_radius(bomb, world)
            for point in radius:
                blast_radius.add(point)

        neighbors = self.get_neighbors((x,y),world)

        if len(all_bombs) > 0:
            farthest = (x, y)
            distance = 0
            for neighbor in neighbors:
                for issue in blast_radius:
                    if neighbor == issue:
                        break
                if not world.empty_at(neighbor[0], neighbor[1]):
                    continue

                benefit = 0
                benefit = self.distance_to_monster(world, neighbor[0], neighbor[1]) + self.distance_to_bomb(world,
                                                                                                            neighbor[0],
                                                                                                neighbor[1])
                for secnd_neighbor in self.get_neighbors(neighbor,world):
                    for issue in blast_radius:
                        if neighbor == issue:
                            break
                    if not world.empty_at(neighbor[0], neighbor[1]):
                        continue

                    benefit += self.distance_to_monster(world, secnd_neighbor[0], secnd_neighbor[1])*10/8
                    benefit += self.distance_to_bomb(world,secnd_neighbor[0],secnd_neighbor[1])/8


                if benefit > distance:
                    distance = benefit
                    farthest = neighbor
                    print(farthest)
                    print(benefit)
            return (farthest[0]-x, farthest[1]-y, False)

        bomb_target_zone = set()
        if closest_char[0] != -1:
            my_neighbors = set()
            for n in self.get_neighbors(closest_char,world):
                my_neighbors.add(n)
            their_neighbors = set()
            for neighbor in my_neighbors:
                for n in self.get_neighbors(neighbor,world):
                    their_neighbors.add(n)
            for n in their_neighbors.difference(my_neighbors):
                if n == closest_char:
                    continue
                bomb_target_zone.add(n)
        else:
            return (0,0, False)

        #Find viable targets
        viable = set()
        backup = set ()
        for target in bomb_target_zone:
            if world.empty_at(target[0], target[1]):
                viable.add(target)

        bomb_target_zone = viable
        closest_target = (-1,-1)
        closest_target_path = []

        for target in bomb_target_zone:
            t_x = target[0]
            t_y = target[1]
            target_path = self.pathfinding((x,y),(t_x,t_y),world)

            if len(closest_target_path) == 0:
                closest_target=target
                closest_target_path=target_path

            if len(target_path) < len(closest_target_path):
                closest_target_path=target_path
                closest_target=target


        #If you are close enough place a bomb
        if self.get_distance((x,y),closest_char) <= 2 or len(closest_target_path) < 2:
            for neighbor in neighbors:
                if neighbor[0] - 1 == x and neighbor[1] - 1 == y:
                    return (neighbor[0]-x, neighbor[1]-y, True)
                elif neighbor[0] + 1 == x and neighbor[1] + 1 == y:
                    return (neighbor[0]-x, neighbor[1]-y, True)
                elif neighbor[0] - 1 == x and neighbor[1] + 1 == y:
                    return (neighbor[0]-x, neighbor[1]-y, True)
                elif neighbor[0] + 1 == x and neighbor[1] - 1 == y:
                    return (neighbor[0]-x, neighbor[1]-y, True)

        #If the exit is unreachable and there is no path
        # if len(closest_target_path) == 0:
        #
        #     #Find the space that is as close as you can get
        #     self.next_nodes.append(closest_target)
        #     self.visited.add(closest_target)
        #     next_best = self.find_next_best(x, y, self.next_nodes[0], world, world.height() * world.width())
        #
        #     #Set the path to the close enough path
        #     closest_target_path = self.pathfinding((x,y), next_best[0], wrld)
        #     closest_target = next_best[0]

        if len(closest_target_path) == 0:
            return(0,0,False)


        #Check if any of the neighbors are in the path
        for neighbor in neighbors:
            for point in closest_target_path:
                if neighbor[0] == point[0] and neighbor[1] == point[1]:
                    #If it is a bomb or an explosion, stay where you are
                    #When you place a bomb, purposely chose a diagonal direction to wait
                    if world.explosion_at(neighbor[0], neighbor[1]) or world.bomb_at(neighbor[0],
                                                                                     neighbor[1]):
                        return (0, 0, False)
                    #Go there
                    return (neighbor[0]-x, neighbor[1]-y, False)


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

    def find_next_best(self, x, y, target, world, path):
        """Find the next best space to an inaccessable target space"""

        #self.next_nodes holds nodes to check if there is an accessable path
        #self.visited hold nodes that have already been checked

        #View the current node
        self.visited.add(target)

        #Check if this is an accessable point
        this_path = self.pathfinding((x, y), target, world)

        #If it is, then return it
        if len(this_path) != 0:
            return (target, len(this_path))


        #Try and compare the shortest paths we find from here
        min_path = world.height()*world.width()
        min_point = (-1,-1)

        #If there are no next nodes, add all of the neighbors of the current neighbors
        #Essentially generation of nodes
        if self.next_nodes == []:
            surrounding = self.get_neighbors(target, world)
            for point in surrounding:
                length = len(self.visited)
                self.visited.add(point)
                if len(self.visited) > length:
                    self.next_nodes.append(point)

        #For all of these next nodes
        for node in self.next_nodes:
            this_node = node

            #If there is a wall in the way, add an extra point to this path
            if world.wall_at(node[0],node[1]):
                path += 1

            #This node not being used
            self.next_nodes.pop(0)

            #Get the path if there is one
            found = self.find_next_best(x, y, this_node, world, path)

            #Get the point with shortest path in this 'generation'
            if found[1] < min_path:
                min_path = found[1]
                min_point = found[0]

        #Return the point that is closest to the exit and to the starting position
        return (min_point, min_path)


    def pathfinding(self, start, end, world):
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

    def get_heuristic(self, start, end):  # for now just compute distance
        """return the heuristic value from start point to end point"""
        return self.get_distance(start, end)

    def get_distance(self, start, end):  # compute distance
        """return Manhatten distance from start to end point"""
        # return math.sqrt(math.pow(start[0] - end[0], 2) + math.pow(start[1] - end[1], 2))
        (x1,y1) = start[0], start[1]
        (x2,y2) = end[0], end[1]
        return abs(x1 - x2) + abs(y1 -y2)

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

        return length

    def distance_to_bomb(self, wrld, x, y):
        """check position of monster relative to character"""
        bombs = self.find_bombs(wrld)

        if len( bombs) == 0:
            return 0
        closest_b = bombs[0]

        for bomb in bombs:
            d1 = self.get_distance((x, y),bomb)
            d2 = self.get_distance((x, y), closest_b)
            if d1 < d2:
                closest_b = bomb

        path = self.pathfinding((x, y), closest_b, wrld)
        length = len(path)
        if length == 0:
            return 0

        return length

    # These are our features I think
    def closest_bombs(self, wrld, x, y):
        """check position of monster relative to character"""
        bombs = self.find_bombs(wrld)

        if len(bombs) == 0:
            return 0
        closest_b = bombs[0]

        for bomb in bombs:
            d1 = self.get_distance((x, y), bomb)
            d2 = self.get_distance((x, y), closest_b)
            if d1 < d2:
                closest_b = bomb

        path = self.pathfinding((x, y), closest_b, wrld)
        return closest_b

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