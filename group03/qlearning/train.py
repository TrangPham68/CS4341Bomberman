# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
import csv
from game import Train, Game
from monsters.selfpreserving_monster import SelfPreservingMonster

# TODO This is your code!
sys.path.insert(1, '../group03/qlearning')
from qlearning import QAgent

for ep in range(2000):
    print("Episode:", ep)

# Create the game
    with open('weights.csv') as csvfile:
        rd = csv.reader(csvfile)
        weights = {rows[0]:float(rows[1]) for rows in rd}

    t = Train.fromfile('training_map.txt')
    t.add_monster(SelfPreservingMonster("aggressive", # name
                                        "A",          # avatar
                                        3, 9,        # position
                                        4             # detection range
    ))

    # TODO Add your character
    maboi = QAgent("me", "C", 0, 0, weights)
    t.add_character(maboi)

    # Run!
    # t.train(0)
    t.train(1)
    with open('weights.csv', 'w') as csvfile:
        w = csv.writer(csvfile)
        for k, v in maboi.weights.items():
            w.writerow([k,v])
