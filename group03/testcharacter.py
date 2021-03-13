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


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

class TestCharacter(CharacterEntity):
    def __init__(self, name, player, x, y):
        CharacterEntity.__init__(self, name, player, x, y)
        self.exit = None
        self.weights = self.weights = np.array([0 for x in range(2)])
        self.learning_rate = 0.2
        self.discount_factor = 0.8
        self.maxdepth = 1
    def do(self, wrld):
        # Your code here
        print (self.x, " ", self.y)
        # if self.exit is None:
        #     x, y = self.find_exit(wrld)
        # path = self.pathfinding((self.x, self.y), (x,y), wrld) #hard code end point
        #
        #if (len(path) > 1) and len(self.find_monsters(wrld)) == 0:
        #      self.move(path[1][0] - self.x, path[1][1] - self.y)
        # else:
        #     move = self.q_learn(wrld, self.x, self.y)
        #     self.update_q(wrld, self.x, self.y)
        grid = self.build_grid(wrld)
        print(grid)
        move = self.expectimax(wrld, grid, self.x, self.y, 0, wrld.time)[0]
        self.move(move[0]-self.x, move[1]-self.y)

    #TODO: COMMENT
    def expectimax(self, world, map, x, y, depth, time):

        grid = map[0]
        exit = map[1]

        st = (y,x)
        end = (18,7)

        print(grid)
        print(st)
        print(end)


        print("\nStart Expectimax at")
        print(x,y)
        #TODO: DEPTH MULTIPLIER
        # TODO: CHANGE TO ACTUAL EXIT
        path = astar(grid, st, end)
        value = -len(path)
        print("To Exit")
        print(value)
        if len(path) == 0:
            return (x,y,0)

        print("To Monster")
        print(self.distance_to_monster(grid, world, x, y))
        value -= (self.distance_to_monster(grid, world, x, y))

        if depth == self.maxdepth or (x,y)==self.find_exit(world):
            return ((x,y), value)

        possible = self.get_neighbors([x,y], world)
        free = []
        for point in possible:
            if world.grid[point[0]][point[1]] == False:
                free.append(point)
        original = world.grid
        max_pt = (-1,-1)
        max_val = -999999999999999

        for point in free:
            world.grid = original
            print("TRYING")
            print(point)

            monster_pos = self.find_monsters(world)
            possible_monster = self.get_neighbors(monster_pos[0], world)
            freemon = []

            for pt in possible_monster:
                print(world.grid[pt[0]][pt[1]])
                if world.grid[pt[0]][pt[1]] == False:
                    freemon.append(pt)


            monster_val = 0
            monster_max = -999999

            for point_2 in freemon:
                print("WITH MONSTER")
                print(point_2)
                world.grid = original
                mon_x = monster_pos[0][0]
                mon_y = monster_pos[0][1]

                world.grid[x][y] = False
                world.grid[point_2[0]][point_2[1]] = True
                world.grid[x][y] = False
                world.grid[point[0]][point[1]] = True

                monster_val += self.expectimax(world, point[0], point[1], depth+1, time-1)[1]
                print("MONSTER VAL")
                print(monster_val)
                world.grid[x][y] = True
                world.grid[point_2[0]][point_2[1]] = False
                world.grid[x][y] = True
                world.grid[point[0]][point[1]] = False

            world.grid = original
            print("SO ALL TOGETHER")
            print(value + (monster_val/len(freemon)))

            if value + (monster_val/len(possible_monster))> max_val:
                max_pt = point
                max_val = value + (monster_val/len(possible_monster))
                print("It's the new max")
                print(max_pt)
                print(max_val)

        new_pt = (max_pt[0],max_pt[1])

        print("FINAL CHOICE")
        print(new_pt)
        print(' ')
        return (new_pt, max_val)



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

    def build_grid(self, world):
        rows, cols = (world.height(),world.width())
        print(world.height())
        print(world.width())
        arr = []
        e=(-1,-1)
        for x in range(rows):
            col = []
            for y in range(cols):
                if world.characters_at(y, x):
                    col.append(1)
                elif world.monsters_at(y, x):
                    col.append(2)
                elif world.wall_at(y, x):
                    col.append(3)
                elif world.bomb_at(y, x):
                    col.append(4)
                elif world.explosion_at(y, x):
                    col.append(5)
                elif world.exit_at(y, x):
                    e = (y,x)
                    col.append(0)
                else:
                    col.append(0)
            arr.append(col)
        return (arr, e)

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
            path.insert(0, current.gtNodePos())
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


    def find_exit(self, wrld): 
        """finds the position of the exit"""
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if wrld.exit_at(x,y):
                    return x,y

    def find_monsters(self, grid, width, height):
        """finds position of nearest monster in world"""
        monsters = []

        for x in range(height):
            for y in range(width):
                if grid[x][y] == 2:
                    monsters.append((x,y))
        print(monsters)
        return monsters

    #These are our features I think
    def distance_to_monster(self, grid, wrld, x, y):
        """check position of monster relative to character"""
        monsters = self.find_monsters(grid, wrld.width(), wrld.height())

        if len(monsters) == 0:
            return 0 
        closest_m = monsters[0]
        
        for monster in monsters:
            d1 = self.get_distance((x,y), monster) 
            d2 = self.get_distance((x,y), closest_m)
            if d1 < d2:
                closest_m = monster 
        print("WHAT")
        path = astar(grid, closest_m, (x,y))
        length = len(path)
        if length == 0:
            return 1
        return length