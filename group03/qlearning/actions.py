from enum import Enum

class Actions(Enum):
    STAY = 0
    N = 1
    NE = 2
    E = 3
    SE = 4
    S = 5
    SW = 6
    W = 7
    NW = 8
    BOMB = 9

class Pos(Enum):
    STAY = (0,0)
    N = (0,-1)
    NE = (1,-1)
    E = (1,0)
    SE = (1,1)
    S = (0,1)
    SW = (-1,1)
    W = (-1,0)
    NW = (-1,-1)
    BOMB = (0,0)
