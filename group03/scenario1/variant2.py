
# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
import csv
from game import Game, Train
from monsters.stupid_monster import StupidMonster

# TODO This is your code!
sys.path.insert(0, '../../group03/qlearning')
from qlearner import QAgent

with open('../qlearning/weights.csv') as csvfile:
    rd = csv.reader(csvfile)
    weights = {rows[0]:float(rows[1]) for rows in rd}

win = 0

for i in range(1):
    # Create the game
    random.seed(i) # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')

    g.add_monster(StupidMonster("stupid", # name
                                "S",      # avatar
                                3, 9      # position
    ))

    # TODO Add your character
    maboi = QAgent("me", "C", 0, 0, weights)
    g.add_character(maboi)

    # Run!
    g.go(1)
    maboi.update_weights(g.world, None)
    if g.done() and g.world.scores["me"] > 0:
        win += 1

print("WIN RATE: ", win, " OUT OF", i+1)

# for k in range(100):
#     # Create the game
#     with open('../qlearning/weights.csv') as csvfile:
#         rd = csv.reader(csvfile)
#         weights = {rows[0]: float(rows[1]) for rows in rd}
#
#     t = Train.fromfile('map.txt')
#     t.add_monster(StupidMonster("stupid", # name
#                                 "S",      # avatar
#                                 3, 9      # position
#     ))
#
#     # TODO Add your character
#     maboi = QAgent("me", "C", 0, 0, weights)
#     t.add_character(maboi)
#
#     # Run!
#     t.train(1)
#     # t.train(1)
#     with open('weights.csv', 'w') as csvfile:
#         w = csv.writer(csvfile, lineterminator='\n')
#         for k, v in maboi.weights.items():
#             w.writerow([k, v])