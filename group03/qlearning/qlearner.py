# This is necessary to find the main code
import math
import sys
import numpy as np
import random 

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
from sensed_world import SensedWorld

sys.path.insert(1, '../bomberman/group03')
import qfunctions as qf

# Instead of epsilon greedy, try using annealing(?)
class QAgent(CharacterEntity):
    def __init__(self, name, player, x, y, weights):
        CharacterEntity.__init__(self, name, player, x, y) 
        self.exit = None 
        self.learning_rate = 0.5
        self.discount_factor = 0.8
        self.epsilon = 0.3
        self.current_state = None
        self.current_action = (0,0)
        self.weights = weights
        self.current_pos = (0,0)

    def do(self, wrld):
        if self.exit is None: 
            x, y = qf.find_exit(wrld)

        path = qf.astar((self.x, self.y), (x,y), wrld) 
        
        # If there are no monsters, use astar
        if (len(path) > 1) and len(qf.find_monsters(wrld)) == 0:
            self.move(path[1][0] - self.x, path[1][1] - self.y)
        else: # Move according to qlearning
            # Save current state
            self.current_state = wrld.from_world(wrld)
            self.current_pos = (self.x, self.y)

            # Find and move with best move
            move = self.get_action(wrld) 
            self.current_action = move
            self.move(move[0], move[1])

            # Place bomb if monster or walls are nearby
            if (qf.monster_within_radius(wrld, self.x, self.y) > 0 or qf.find_walls(wrld, self.x, self.y) >= 3):
                self.place_bomb()


    def get_legal_actions(self, wrld, curr_pos):
        """get possible actions given current position"""
        neighbors = qf.get_neighbors(wrld, curr_pos) 
        directions = [] 
        for n in neighbors:
            if wrld.empty_at(n[0], n[1]):
                directions.append((n[0] - curr_pos[0], n[1] - curr_pos[1])) 
        
        return directions

    def extract_features(self, wrld, x, y):
        """get list of feature values"""
        features = {}
        features['dist_to_monsters'] = qf.distance_to_monster(wrld, x, y)
        features['dist_to_exit'] = qf.distance_to_exit(wrld,x,y)
        features['dist_to_bomb'] = qf.distance_to_bomb(wrld, x, y)
        # features['m_within_rad'] = qf.monster_within_radius(wrld, x, y)
        features['blast'] = qf.blast_radius(wrld, x, y)
        features['cornered'] = qf.if_cornered(wrld, x, y)
        return features

    def q_value(self, wrld, x, y):
        """find the qvalue of a state-action pair"""
        q = 0 
        fvec = self.extract_features(wrld, x, y) 
        for f in fvec: 
            q += self.weights[f] * fvec[f] 
        return q

    def get_qmax(self, wrld, x, y):
        """find the max qvalue within state-action pairs"""
        legal_a = self.get_legal_actions(wrld, (x, y)) 
        if len(legal_a) == 0:
            return 0

        q_table = [] 

        for a in legal_a: 
            q_table.append(self.q_value(wrld, x + a[0], y + a[1]))
            print("Q_table:", q_table)
        return max(q_table)

    def get_best_action(self, wrld):
        """return the best action given state-action pairs"""

        try:
            char = list(wrld.characters.values())[0][0]
            print("Charcter:", char)
            char_pos = (char.x, char.y)

            legal_a = self.get_legal_actions(wrld, char_pos)
        except:
            print("errrr")
            legal_a = []

        
        # There could be more than one best action, but we pick the first one
        best_action = (0,0)

        monsters = wrld.monsters.values()

        if len(legal_a) > 0:
            for a in legal_a:
                # Set move in Sensed World
                wrld.me(self).move(a[0], a[1])
                # make move
                if len(monsters) == 1:
                    m = next(iter(monsters))[0]
                elif len(monsters) == 2:
                    monster = iter(wrld.monsters.values())
                    m1 = next(monster)[0]
                    m2 = next(monster)[0]
                    m_pos_list = [(m1.x, m1.y), (m2.x, m2.y)]
                    m_closest = qf.find_closest_obj(wrld, m_pos_list, char.x, char.y)
                    if m_closest == m_pos_list[0]:
                        m = m1
                    else:
                        m = m2
                # Go through possible actions for monster
                m_pos = (m.x, m.y)
                m_legal_a = self.get_legal_actions(wrld, m_pos)
                len_m_path = 0
                m_optimal_step = (0,0)
                qmax = 0
                for ma in m_legal_a:
                    #Finding optimal monster move
                    path = qf.astar((ma[0],ma[1]), (char.x, char.y), wrld)
                    if len(path) > len_m_path:
                        len_m_path = len(path)
                        m_optimal_step = ma
                
                # Set monster move in Sensed World
                m.move(m_optimal_step[0], m_optimal_step[1])
                new_wrld, events = wrld.next()

                new_q = self.q_value(new_wrld, char.x + a[0], char.y + a[1])
                print("New q", new_q)
                if new_q > qmax:
                    qmax = new_q
                    best_action = a

                print("Best actions:", best_action)
    
        return best_action

    def get_action(self, wrld):
        """take action with epsilon-greedy implementation"""

        new_action = None

        for val in wrld.characters.values():
            if len(val) > 0:
                char = val[0] 
                char_pos = (char.x, char.y)
                legal_a = self.get_legal_actions(wrld, char_pos)

                rand = random.random()
                
                if (rand < self.epsilon):
                    new_action = random.choice(legal_a)
                else:
                    new_action = self.get_best_action(wrld)
                
        return new_action

    def eval_state(self, wrld, curr_pos):
        curr_x, curr_y = curr_pos[0], curr_pos[1]
        mdist = 1 / math.sqrt(qf.distance_to_monster(wrld, curr_x, curr_y))
        edist = 1 / math.sqrt(qf.distance_to_exit(wrld, curr_x, curr_y))

        print("Mdist, edist:", mdist, edist)
        return abs(mdist)/abs(edist)

    def calc_rewards(self, wrld, events):
        """
        Loop over heuristics function and evaluate at current worldstate
        return sum of heuristics
        """
        sum = 0
        if len(events) > 0:
            for event in events:
                if event.tpe == event.CHARACTER_FOUND_EXIT:
                    sum += 200
                elif event.tpe == event.BOMB_HIT_CHARACTER or event.tpe == event.CHARACTER_KILLED_BY_MONSTER:
                    sum += -100
        else:
            sum += self.eval_state(wrld, (self.current_pos[0], self.current_pos[1]))

        return sum

    def update_weights(self, new_state, events):
        """update feature weights"""
        dx, dy = self.current_action[0], self.current_action[1]
        reward = self.calc_rewards(self.current_state, events) 
        current_q = self.q_value(self.current_state, self.current_pos[0], self.current_pos[1])
        # Create SensedWorld from Real World
        next_state = SensedWorld.from_world(new_state)

        # Get best possible action from next state along with position of character
        next_action = self.get_best_action(next_state)

        next_pos = (self.current_pos[0] + dx + next_action[0], self.current_pos[1] + dy + next_action[1])

        print("Old world, new world:",  self.current_state, next_state)
        print("Initial position:", self.current_pos, "Next position:", (self.current_pos[0] + dx, self.current_pos[1] + dy))
        print("Position after:", next_pos)
        print("Current action:", self.current_action, "Next action:", next_action)

        # delta = r + v(max(a')(Q(s',a'))) - Q(s,a)
        delta = (reward + (self.discount_factor * self.q_value(next_state, next_pos[0], next_pos[1]))) - current_q

        fvec = self.extract_features(self.current_state, self.current_pos[0], self.current_pos[1])
        for f in fvec: 
            self.weights[f] = self.weights[f] + self.learning_rate  * delta * fvec[f]