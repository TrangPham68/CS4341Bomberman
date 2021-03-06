sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import random 
import numpy as np
from sensed_world import SensedWorld 

class qLearningAgent(CharacterEntity):

    def __init__(self, name, player, x, y):
        CharacterEntity.__init__(self, name, player, x, y) 
        self.exit = None 
        self.weights = np.array([0 for x in range(3)])
        self.learning_rate = 0.2
        self.discount_factor = 0.8
        self.all_moves = [(1, 0, 0), (1, 1, 0), (1, -1, 0), (-1, 0, 0), (-1, 1, 0), (-1, -1, 0), (0, -1, 0), (0, 1, 0), (0, 0, 0),
                              (1, 0, 1), (1, 1, 1), (1, -1, 1), (-1, 0, 1), (-1, 1, 1), (-1, -1, 1), (0, -1, 1), (0, 1, 1), (0, 0, 1)]
    def getQValue(self, s, a):
        #some sort of feature vec? 

    