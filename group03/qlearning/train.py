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

maps = ['training_maps/1.txt','training_maps/2.txt','training_maps/3.txt','training_maps/4.txt','training_maps/5.txt', '../scenario1/map.txt', '../scenario2/map.txt']
winTrial = []
winRate = []
winCount = 0
for i in range(1): #5
    for j in range(1): #5
        winCount = 0
        for k in range(500): #20
            # Create the game
            with open('weights4.csv') as csvfile:
                rd = csv.reader(csvfile)
                weights = {rows[0]:float(rows[1]) for rows in rd}

            t = Train.fromfile(maps[5])
            if i % 2 == 0:
                t.add_monster(SelfPreservingMonster("aggressive", # name
                                                    "A",          # avatar
                                                    4, 7,        # position
                                                    2             # detection range
                ))
            else:
                t.add_monster(StupidMonster("stupid",  # name
                                        "S",  # avatar
                                        3, 9  # position
                                        ))

            # TODO Add your character
            maboi = QAgent("me", "C", 0, 0, weights)
            t.add_character(maboi)

            # Run!
            t.train(1)
            maboi.update_weights(t.world, None)

            if t.done() and t.world.scores["me"] > 0:
                winCount += 1
            # t.train(1)
            with open('weights4.csv', 'w') as csvfile:
                w = csv.writer(csvfile, lineterminator='\n')
                for k, v in maboi.weights.items():
                    w.writerow([k,v])

        winRate.append((j, winCount))
    winTrial.append((i, winRate))

for win in winTrial:
    print("Trial: ", win[0], " Map:", win[1][0][0], " Win Count:", win[1][0][1], "over ", 20, "runs")