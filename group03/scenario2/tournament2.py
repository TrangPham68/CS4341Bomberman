# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
from game import Game, Train
import random
import csv
from monsters.selfpreserving_monster import SelfPreservingMonster
from monsters.stupid_monster import StupidMonster
sys.path.insert(1, '../../group03/qlearning')
from qlearner import QAgent


# TODO This is your code!

# Create the game
winRate = []
for game in range (5):
	for variant in range(5):
		win = 0;
		for trial in range(100):
			random.seed(trial)
			with open('tourWeight.csv') as csvfile:
				rd = csv.reader(csvfile)
				weights = {rows[0]: float(rows[1]) for rows in rd}

			g = Train.fromfile('map.txt')
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

			with open('tourWeight.csv', 'w') as csvfile:
				w = csv.writer(csvfile, lineterminator='\n')
				for k, v in maboi.weights.items():
					w.writerow([k, v])






# # TODO Add your character
# g.add_character(TestCharacter("me", # name
#                               "C",  # avatar
#                               0, 0  # position
# ))

# Run!
# g.go(1)
