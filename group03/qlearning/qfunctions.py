# This is necessary to find the main code
import math
import sys
import node

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from colorama import Fore, Back
from sensed_world import SensedWorld
from queue import PriorityQueue

#FEATURES

# Find distance from character to nearest monster, normalized
def distance_to_monster(wrld, x, y):
    """check position of monster relative to character"""
    monsters = find_monsters(wrld) 
    closest_m = find_closest_obj(wrld, monsters, x, y)

    # If there is no monster 
    if closest_m == 0:
        return 0.0
    
    if (x >= 0) and (x < wrld.width()):
        # Avoid out-of-bound indexing
        if (y >= 0) and (y < wrld.height()):
            path = astar((x,y), closest_m, wrld)
            length = len(path)

            return 1.0/(length + 1)

    # Return a high cost if out of bounds
    return 1.0

# Find distance from character to exit, normalized
def distance_to_exit(wrld, x, y):
    """check distance to exit"""
    exit_loc = find_exit(wrld) 
    if (x,y) == exit_loc:
        return 0.0

    if (x >= 0) and (x < wrld.width()):
        # Avoid out-of-bound indexing
        if (y >= 0) and (y < wrld.height()):
            path = astar((x,y), exit_loc, wrld)
            length = len(path) 

            return 1.0/(length + 1) 

    # Return a high cost if out of bounds
    return 1.0

# # Find distance from character to closest bomb, normalized
# def bomb_radius(wrld, x, y):
#     """
#     Checks if player is within potential explosion radius
#     """
#     for dx in range(5,6):
#         if (x + dx >= 0) and (x + dx < wrld.width()):
#             if wrld.bomb_at(x + dx, y):
#                 return 1.0
#     for dy in range(-5, 6):
#         if (y + dy >= 0) and (y + dy < wrld.height()):
#             if wrld.bomb_at(x, y + dy):
#                 return 1.0
#     return 0.0

def distance_to_bomb(wrld,x,y):
    """
    Checks the number of moves from character to bomb
    Returns length of astar path, normalized
    """
    bombs = find_bombs(wrld)
    closest_b = find_closest_obj(wrld, bombs, x, y)

    # If there is no monster 
    if closest_b == 0:
        return 0.0
    
    if (x >= 0) and (x < wrld.width()):
        # Avoid out-of-bound indexing
        if (y >= 0) and (y < wrld.height()):
            path = astar((x,y), closest_b, wrld)
            length = len(path)

            return 1.0/(length + 1)

    # Return a high cost if out of bounds
    return 1.0

def blast_radius(wrld, x, y):
    """Check if agent and future moves is within explosion"""
    for dx in [-1,0,1]:
        # Avoid out-of-bound indexing 
        if (x + dx >= 0) and (x + dx < wrld.width()):
            # Loop through y directions
            for dy in [-1,0,1]:
                # Avoid out-of-bound indexing
                if (y + dy >= 0) and (y + dy < wrld.height()):
                    for i in range(-1, 1):
                        for j in range(-1,1):
                            if wrld.explosion_at(x + dx + i, y + dy + j):
                                return 1.0

    return 0.0

def monster_within_radius(wrld, x, y):
    """Check if monster is within 3 tiles away"""
    monsters = find_monsters(wrld) 
    closest_m = find_closest_obj(wrld, monsters, x, y)
    if closest_m == 0:
        return 0.0
    
    if (x >= 0) and (x < wrld.width()):
        # Avoid out-of-bound indexing
        if (y >= 0) and (y < wrld.height()):
            path = astar((x,y), closest_m, wrld)
            length = len(path)
            if length <= 3:
                return float(4 - length / 4) # put emphasis on nearer monsters
    
    return 0.0

# def if_cornered(wrld, x, y):
#     """check if agent is cornered and has less than 3 possible moves"""
#     x, y = (curr_pos[0], curr_pos[1])
#         directions = []

#         for c_dx in [-1,0,1]:
#             # Avoid out-of-bound indexing 
#             if (x + c_dx >= 0) and (x + c_dx < wrld.width()):
#                 # Loop through y directions
#                 for c_dy in [-1,0,1]:
#                     # Avoid out-of-bound indexing
#                     if (y + c_dy >= 0) and (y + c_dy < wrld.height()):
#                         # No need to check impossible moves
#                         if not wrld.wall_at(x + c_dx, y + c_dy) or not wrld.bomb_at(x + c_dx, y + c_dy):
#                             # Set move in Sensed World
#                             directions.append((c_dx, c_dy))
        
#         return len(directions) < 3

def find_walls(wrld, x, y):
    """find number of surrounding walls"""
    walls = []
    for dx in [-1, 0, 1]:
        if (x + dx >= 0) and (x + dx < wrld.width()):
            for dy in [-1, 0, 1]:
                if not((dx == 0) and (dy == 0)):
                    if (y + dy >= 0) and (y + dy < wrld.height()):
                        if wrld.wall_at(x + dx, y + dy):
                            walls.append((x + dx, y + dy))
    return len(walls)

def create_node(pos):
    """create a Node of a specific location"""
    return node.Node(pos)

def get_heuristic(start, end):  # for now just compute distance
    """return the heuristic value from start point to end point"""
    return get_distance(start, end)  

def get_distance(start, end):  # compute distance
    """return Manhatten distance from start to end point"""
    # return math.sqrt(math.pow(start[0] - end[0], 2) + math.pow(start[1] - end[1], 2))
    (x1,y1) = start[0], start[1]
    (x2,y2) = end[0], end[1]
    return abs(x1 - x2) + abs(y1 -y2)

def get_path(startNode, endNode):
    """return the list of (x, y) point from start to end by backtracking parent node from end"""
    current = endNode
    path = []

    while (not current == startNode):
        path.insert(0, current.getNodePos())
        current = current.getParent()

    path.insert(0, startNode.getNodePos())
    return path

def get_neighbors(wrld,pos):
    """get all the possible neighbor location and return a set of (x,y) neighbors"""
    width = wrld.width()
    height = wrld.height()
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
        
def astar(start, end, world):  # start (x,y) and end (x,y)
    """Apply Astar to find the closest path from start to end in world"""
    startNode = create_node(start)
    endNode = create_node(end)
    visited = {}

    seenNeighbor = {}

    frontier = PriorityQueue()
    startNode.setCostSoFar(0)
    startNode.setEstCost(get_heuristic(start, end))
    frontier.put(startNode)
    visited[start] = 0

    seenNeighbor[start] = startNode

    while (not frontier.empty()):
        next = frontier.get()
        #visited[next.getNodePos()] = next

        if next.getNodePos() == endNode.getNodePos():
            return get_path(startNode, next)
        neighbor = get_neighbors(world, next.getNodePos())
        for i in neighbor:
            # if wall, ignore
            if (world.wall_at(i[0], i[1])):
                continue

            if not i in seenNeighbor:
                node = create_node(i)
                seenNeighbor[i] = node
            else:
                node = seenNeighbor.get(i)


            if (next.getParent() == None or not (node == next.getParent())):
            # if the node is not next parents -> avoid duplicate path
                if (next.getCostSoFar() == math.inf):
                    next.setCostSoFar(0)
                cost = get_distance(next.getNodePos(), node.getNodePos()) + next.getCostSoFar()

                if (cost < node.getCostSoFar()):
                    node.setParent(next)
                    node.setCostSoFar(cost)
                    node.setEstCost(cost + get_heuristic(node.getNodePos(), endNode.getNodePos()))
                    frontier.put(node)

    return [] #return empty if we find no path

def find_exit(wrld): 
    """finds the position of the exit"""
    for x in range(wrld.width()):
        for y in range(wrld.height()):
            if wrld.exit_at(x,y):
                return x,y

def find_monsters(wrld):
    """finds position of nearest monster in world"""
    monsters = []
    for x in range(wrld.width()):
        for y in range(wrld.height()):
            if wrld.monsters_at(x,y):
                monsters.append((x,y))
    
    return monsters

def find_bombs(wrld):
    """find all bombs in world"""
    bombs = []
    for x in range(wrld.width()):
        for y in range(wrld.height()):
            if wrld.bomb_at(x,y):
                bombs.append((x,y))
    return bombs

def find_blasts(wrld):
    """find all bombs in world"""
    explosions = []
    for x in range(wrld.width()):
        for y in range(wrld.height()):
            if wrld.explosion_at(x,y):
                explosions.append((x,y))
    
    return explosions

def find_closest_obj(wrld, objs, x, y):
    """find closest object to character"""

    if len(objs) == 0:
        return 0 
    closest_obj = objs[0]
    
    for o in objs:
        d1 = get_distance((x,y), o) 
        d2 = get_distance((x,y), closest_obj)

        # If d1 is closer than d2
        if d1 < d2:
            closest_obj = o

    return closest_obj