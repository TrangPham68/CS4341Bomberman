# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
import csv
from monsters.selfpreserving_monster import SelfPreservingMonster
from monsters.stupid_monster import StupidMonster
from game import Game
from testcharacter import TestCharacter

# TODO This is your code!
sys.path.insert(1, '../group03/qlearning')

maps = ['training_maps/1.txt','training_maps/2.txt','training_maps/3.txt','training_maps/4.txt','training_maps/5.txt']

for m in range(5):
    for k in range(100):
        # Create the game
        g = Game.fromfile(maps[m])
        g.add_monster(SelfPreservingMonster("aggressive", # name
                                            "A",          # avatar
                                            4, 7,        # position
                                            2             # detection range
        ))

        # g.add_monster(StupidMonster("stupid", # name
        #                             "S",      # avatar
        #                             1, 6      # position
        # ))

        g.add_character(TestCharacter("me",  # name
                                      "C",  # avatar
                                      0, 0  # position
                                      ))

        # Run!
        g.go(1)