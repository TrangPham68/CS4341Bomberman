# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
import csv
from game import Train, Game
from monsters.selfpreserving_monster import SelfPreservingMonster
from monsters.stupid_monster import StupidMonster

# TODO This is your code!
sys.path.insert(1, '../group03/qlearning')
from qlearner import QAgent

winTrial = []
winRate = []
winCount = 0
for i in range(1000): #5
    winCount = 0
    random.seed(i)
    # Create the game
    with open('weights.csv') as csvfile:
        rd = csv.reader(csvfile)
        weights = {rows[0]:float(rows[1]) for rows in rd}

    t = Train.fromfile('training_maps/6.txt')
    if i % 2 == 0:
        t.add_monster(SelfPreservingMonster("aggressive", # name
                                            "A",          # avatar
                                            3, 13,        # position
                                            2             # detection range
        ))
    elif i % 2 == 1:
        t.add_monster(StupidMonster("stupid",  # name
                                "S",  # avatar
                                3, 9  # position
                                ))

    maboi = QAgent("me", "C", 0, 0, weights)
    t.add_character(maboi)

    # Run!
    t.train(1)
    if maboi.lastReward != -100 and maboi.lastReward != -60:
        maboi.update_weights(t.world, None)

    if t.done() and t.world.scores["me"] > 0:
        winCount += 1
    with open('weights2.csv', 'w') as csvfile:
        w = csv.writer(csvfile, lineterminator='\n')
        for k, v in maboi.weights.items():
                w.writerow([k,v])

    winTrial.append((i, winRate))

for win in winTrial:
    print("Trial: ", win[0], " Map:", win[1][0][0], " Win Count:", win[1][0][1], "over ", 20, "runs")