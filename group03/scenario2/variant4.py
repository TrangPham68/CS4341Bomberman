# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
import csv
from game import Game, Train
from monsters.selfpreserving_monster import SelfPreservingMonster

# TODO This is your code!
sys.path.insert(0, '../../group03/qlearning')
from qlearner import QAgent

with open('../qlearning/weights.csv') as csvfile:
    rd = csv.reader(csvfile)
    weights = {rows[0]:float(rows[1]) for rows in rd}

win = 0
for i in range(5):
    # Create the game
    random.seed(i) # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')
    g.add_monster(SelfPreservingMonster("aggressive", # name
                                        "A",          # avatar
                                        3, 13,        # position
                                        2             # detection range
    ))

    # TODO Add your character
    maboi = QAgent("me", "C", 0, 0, weights)
    g.add_character(maboi)

    # Run!
    g.go(1)
    if g.done() and g.world.scores["me"] > 0:
        win += 1

print("WIN RATE: ", win, " OUT OF", i+1)
