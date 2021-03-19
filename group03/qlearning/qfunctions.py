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

def distance_to_monster(wrld, x, y):
    """# Find distance from character to nearest monster, normalized"""
    m = find_closest_monster(wrld, x, y)

    if not m: # If there is no monster
        return 0 

    path = astar((x,y), (m.x, m.y), wrld)
    length = len(path)
    path = astar((x,y), (m.x, m.y), wrld, False)
    length = len(path)
    # if (length > wrld.height()):
    #     return 0
    cost = 0
    if length > 1:
        #print(path[length - 1][1])
        cost = path[length - 1][1]
    #return 1 / ((0.5 * cost) ** 2 + 1)
    return 1.0 / (cost + 1)

def distance_to_exit(wrld, x, y):
    """Find distance from character to exit, normalized"""
    exit_loc = find_exit(wrld) 
    path = astar((x,y), exit_loc, wrld, False)
    length = len(path)
    cost = 0
    if length > 1:
        #print(path[length-1][1])
        cost = path[length-1][1]
    #return 1 / (0.6*length + 1)

    #return 1 / (0.5 * cost + 1)
    return 1.0 / (cost + 1)

def bomb_radius(wrld, x, y):
    """Find distance from character to closest bomb, normalized"""
    danger = 0
    bombs = list(wrld.bombs.values())

    # If character in explosion
    if wrld.explosion_at(x,y):
        return 1.0

    # If there is a bomb but character is not in an explosion
    if len(bombs) > 0:
        b = bombs[0]
        e_range = expl_radius(wrld, b.x, b.y)

        # Penalize more if character in range and bomb is about to explode
        if (x,y) in e_range:
            return (1.0 / (b.timer + 1)) ** 0.1
    
    # No bombs
    return 0

def if_blocked(wrld, x, y):
    exit_loc = find_exit(wrld)
    path = astar((x,y), exit_loc, wrld)
    idx = 0
    for pos in path:
        #idx += 1
        if (pos[0] >= 0) and (pos[0] < wrld.width()) and (pos[1] >= 0) and (pos[1] < wrld.height()):
            if wrld.monsters_at(pos[0], pos[1]):
                path = astar((x, y), pos, wrld, False)
                return 1 / (path[len(path) - 1][1] + 1)

    return 0

def m_to_bomb(wrld,x,y):
    bombs = list(wrld.bombs.values())
    if len(bombs) > 0:
        b = bombs[0]
        m = find_closest_monster(wrld,x,y)
        if m:
            e_range = expl_radius(wrld,b.x,b.y)
            if (m.x,m.y) in e_range:
                return 1.0
    return 0

# def if_bomb_monster(wrld, x, y):
#     exit_loc = find_exit(wrld)
#     path = astar((x,y), exit_loc, wrld, False) #Astar to exit without minding walls
#     if len(path) > 0:
#         for pos in path:
#             if (pos[0] >= 0) and (pos[0] < wrld.width()) and (pos[1] >= 0) and (pos[1] < wrld.height()):
#                 if wrld.monsters_at(pos[0], pos[1]): # Check if wall or monster is in astar path
#                     # Check if wall is in bomb radius if character places bomb
#                     b_range = expl_radius(wrld, x, y)
#                     if pos in b_range:
#                         return 1.0

#     return 0

def if_bomb_wall(wrld, x, y):
    exit_loc = find_exit(wrld)
    path = astar((x,y), exit_loc, wrld, False) #Astar to exit without minding walls
    b_range = expl_radius(wrld, x, y)
    m = wrld.monsters.values()
    if len(path) > 0 and len(m) > 0: # If we have a path and there are monsters
        for pos in path:
            pos = pos[0]
            if (pos[0] >= 0) and (pos[0] < wrld.width()) and (pos[1] >= 0) and (pos[1] < wrld.height()):
                if wrld.wall_at(pos[0], pos[1]): # Check if wall or monster is in astar path
                    if pos in b_range:
                        return 1.0

    return 0

# def all_alone(wrld,x,y):
#     """Check if agent is all alone in the world"""
#     exit_loc = find_exit(wrld)
#     if len(wrld.monsters.values()) == 0 and len(astar((x,y),exit_loc,wrld)) > 0:
#         return 1.0
#     return 0

# HELPER FUNCTUIONS

def create_node(pos):
    """create a Node of a specific location"""
    return node.Node(pos)

def get_heuristic(start, end, wrld):  # for now just compute distance
    """return the heuristic value from start point to end point"""
    if (wrld.wall_at(start[0] , start[1])):
        return 2 * get_distance(start, end)
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
        path.insert(0, (current.getNodePos(), current.getCostSoFar()))
        current = current.getParent()

    path.insert(0, (startNode.getNodePos(), startNode.getCostSoFar()))
    return path

def get_neighbors(wrld,pos):
    """get all the possible neighbor location and return a set of (x,y) neighbors"""
    neighbor = []
    x = pos[0]
    y = pos[1]

    for dx in [-1, 0, 1]:
        if (x + dx >= 0) and (x + dx < wrld.width()):
            for dy in [-1, 0, 1]:
                if not ((dx == 0) and (dy == 0)):
                    if (y + dy >= 0) and (y + dy < wrld.height()):
                        neighbor.append((x + dx, y + dy))
    
    return neighbor
        
def astar(start, end, world,  ignoreWall = True):  # start (x,y) and end (x,y)
    """Apply Astar to find the closest path from start to end in world"""
    #print(start)
    if (start[0] >= 0) and (start[0] < world.width()) and (start[1] >= 0) and (start[1] < world.height()):
        hi = 0
    else:
        #print("AHHHH")
        return []
    startNode = create_node(start)
    endNode = create_node(end)


    seenNeighbor = {}

    frontier = PriorityQueue()
    startNode.setCostSoFar(0)
    startNode.setEstCost(get_heuristic(start, end, world))
    frontier.put(startNode)
    seenNeighbor[start] = startNode
    path = []

    while (not frontier.empty()):
        next = frontier.get()
        #visited[next.getNodePos()] = next

        if next.getNodePos() == endNode.getNodePos():
            path = get_path(startNode, next)
            break
        neighbor = get_neighbors(world, next.getNodePos())
        for i in neighbor:
            # if wall, ignore
            if (ignoreWall and world.wall_at(i[0], i[1])):
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
                    node.setEstCost(cost + get_heuristic(node.getNodePos(), endNode.getNodePos(), world))
                    frontier.put(node)
    re_path = []
    if (ignoreWall == False):
        return path
    else:
        for step in path:
            re_path.append(step[0])
        return re_path #return empty if we find no path

def find_closest_obj(wrld, objs, x, y):
    """find closest object to character"""

    if len(objs) == 0:
        return 0 
    closest_o = objs[0]
    
    for o in objs:
        # Find distance from (x,y) to object
        d1 = get_distance((x,y), (o.x, o.y))
        d2 = get_distance((x,y), (closest_o.x, closest_o.y))

        # If d1 is closer than d2
        if d1 < d2:
            closest_o = o

    return closest_o

def find_closest_monster(wrld, x, y):
    """
    Finds the monster closest to a given position, if any
    Returns the entity
    """
    m = 0
    monsters = wrld.monsters.values()

    if len(monsters) == 1:
        m = next(iter(monsters))[0]
    elif len(monsters) == 2:
        monster = iter(wrld.monsters.values())
        m1 = next(monster)[0]
        m2 = next(monster)[0]
        m_pos_list = [m1, m2]
        m_closest = find_closest_obj(wrld, m_pos_list, x, y)
        if m_closest == m_pos_list[0]:
            m = m1 
        else:
            m = m2

    return m

def find_exit(wrld): 
    """finds the position of the exit"""
    for x in range(wrld.width()):
        for y in range(wrld.height()):
            if wrld.exit_at(x,y):
                return x,y


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

def expl_radius(wrld, x, y):
    """Return radius of explosion given a bomb's location"""
    e_range = set()

    # Grab coordinates of explosion range
    for dx in range(-wrld.expl_range, wrld.expl_range + 1):
        if (x + dx >= 0) and (x + dx < wrld.width()):
            e_range.add((x + dx, y))
    for dy in range(-wrld.expl_range, wrld.expl_range + 1):
        if (y + dy >= 0) and (y + dy < wrld.height()):
            e_range.add((x, y + dy))

    return e_range