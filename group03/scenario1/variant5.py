# This is necessary to find the main code
import sys
import csv
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.stupid_monster import StupidMonster
from monsters.selfpreserving_monster import SelfPreservingMonster
sys.path.insert(1, '../../group03/qlearning')
from qlearner import QAgent

# TODO This is your code!
sys.path.insert(1, '../groupNN')
#from testcharacter import TestCharacter

# Create the game


with open('../qlearning/weights.csv') as csvfile:
    rd = csv.reader(csvfile)
    weights = {rows[0]: float(rows[1]) for rows in rd}

win =0
for i in range(5):
    random.seed(123)  # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')
    g.add_monster(StupidMonster("stupid", # name
                                "S",      # avatar
                                 3, 5,     # position
    ))
    g.add_monster(SelfPreservingMonster("aggressive", # name
                                        "A",          # avatar
                                         3, 13,        # position
                                        2             # detection range
    ))

    maboi = QAgent("me", "C", 0, 0, weights)
    g.add_character(maboi)
    # Run!
    g.go(1)
    if g.done() and g.world.scores["me"] > 0:
        win += 1

print("WIN RATE: ", win, " OUT OF", i + 1)