# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
from game import Game
import random
import csv
from monsters.selfpreserving_monster import SelfPreservingMonster
from monsters.stupid_monster import StupidMonster
sys.path.insert(1, '../../group03/qlearning')
from qlearner import QAgent


# TODO This is your code!

# Create the game
winRate = []
for variant in range (5):
	win = 0;
	for trial in range (1):
		random.seed(123)
		with open('../qlearning/weights.csv') as csvfile:
			rd = csv.reader(csvfile)
			weights = {rows[0]: float(rows[1]) for rows in rd}

		g = Game.fromfile('map.txt')
		if variant == 1 or variant == 4:
			g.add_monster(StupidMonster("stupid",  # name
			                            "S",  # avatar
			                            3, 5  # position
			))
		if variant == 2:
			g.add_monster(StupidMonster("stupid",  # name
			                            "S",  # avatar
			                            3, 5  # position
			))
		if variant == 3 or variant == 4:
			g.add_monster(SelfPreservingMonster("aggressive",  # name
			                                    "A",  # avatar
			                                    3, 13,  # position
			                                    2  # detection range
			))
		maboi = QAgent("me", "C", 0, 0, weights)
		g.add_character(maboi)
		g.go(1)
		if g.done():
			if maboi.win == True:
				win += 1
	winRate.append((variant, win))

print(winRate)



# # TODO Add your character
# g.add_character(TestCharacter("me", # name
#                               "C",  # avatar
#                               0, 0  # position
# ))

# Run!
# g.go(1)
