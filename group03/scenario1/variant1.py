
# This is necessary to find the main code
import csv
import sys

from game import Game

sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

import random

# TODO This is your code!
sys.path.insert(0, '../../group03/qlearning')
from qlearner import QAgent

with open('../qlearning/weights.csv') as csvfile:
    rd = csv.reader(csvfile)
    weights = {rows[0]:float(rows[1]) for rows in rd}

win = 0

for i in range(100):
    # Create the game
    random.seed(i) # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')

    # TODO Add your character
    maboi = QAgent("me", "C", 0, 0, weights)
    g.add_character(maboi)

    # Run!
    g.go(1)
    win += g.win

print("WIN RATE: ", win, " OUT OF 100")