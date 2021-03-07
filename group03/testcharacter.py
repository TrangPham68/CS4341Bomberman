# This is necessary to find the main code
import math
import sys
from queue import PriorityQueue

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back

class TestCharacter(CharacterEntity):
    def __init__(self, name, player, x, y):
        CharacterEntity.__init__(self, name, player, x, y) 

    def do(self, wrld):
        pass