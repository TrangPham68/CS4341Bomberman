
# This is necessary to find the main code
import sys
import csv
import random
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# TODO This is your code!
sys.path.insert(0, '../../group03/qlearning')
from qlearner import QAgent
from game import Game

with open('../qlearning/weights5.csv') as csvfile:
    rd = csv.reader(csvfile)
    weights = {rows[0]:float(rows[1]) for rows in rd}

win = 0

for i in range(10):
    # Create the game
    random.seed(i) # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')

    # TODO Add your character
    maboi = QAgent("me", "C", 0, 0, weights)
    g.add_character(maboi)

    # Run!
    g.go(1)
    maboi.update_weights(g.world, None)
    if g.done() and g.world.scores["me"] > 0:
        win += 1

print("WIN RATE: ", win, " OUT OF", i+1)