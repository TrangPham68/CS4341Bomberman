# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.stupid_monster import StupidMonster

# TODO This is your code!
sys.path.insert(1, '../groupNN')
from scenario1var2 import ExpectimaxCharacter

# Create the game
random.seed(12) # TODO Change this if you want different random choices
g = Game.fromfile('map.txt')
g.add_monster(StupidMonster("stupid", # name
                            "S",      # avatar
                            1, 6      # position
))

# TODO Add your character
g.add_character(ExpectimaxCharacter("me", # name
                              "C",  # avatar
                              0, 0  # position
))

# # Uncomment this if you want the interactive character
# from interactivecharacter import InteractiveCharacter
#
# # Uncomment this if you want the interactive character
# g.add_character(InteractiveCharacter("me", # name
#                                      "C",  # avatar
#                                      0, 0  # position
# ))

# Run!
g.go(600)
