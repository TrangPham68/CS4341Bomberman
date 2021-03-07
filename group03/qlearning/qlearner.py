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
        self.learning_rate = 0.2
        self.discount_factor = 0.8
        self.epsilon = 0.2
        self.last_action = (0,0) 
        self.last_state = None
        self.weights = weights

    def do(self, wrld):
        """main q learning"""
        if self.exit is None: 
            x, y = qf.find_exit(wrld)

        if not bool(self.weights):
            self.weights = self.extract_features(wrld, self.x, self.y)

        path = qf.astar((self.x, self.y), (x,y), wrld) 
        
        # If there are no monsters, use astar
        if (len(path) > 1) and len(qf.find_monsters(wrld)) == 0:
            self.move(path[1][0] - self.x, path[1][1] - self.y)
        else: # Move according to qlearning
            move = self.get_action(wrld) 
            self.last_state = wrld
            self.move(move[0], move[1])
            self.update_weights(self.last_state, wrld, self.x, self.y)
            # Place bomb if monster or walls are nearby
            if (qf.monster_within_radius(wrld, x, y) > 0 or qf.find_walls(wrld, x, y) >= 3):
                self.place_bomb()
            self.last_action = move
            print("Weights:", self.weights)

    def get_legal_actions(self, wrld):
        """get possible actions given current position"""
        curr_pos = (self.x, self.y)
        neighbors = qf.get_neighbors(wrld, curr_pos) 
        directions = [] 
        for n in neighbors:
            if wrld.empty_at(n[0], n[1]):
                directions.append((n[0] - self.x, n[1] - self.y)) 
        
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
        legal_a = self.get_legal_actions(wrld) 
        if len(legal_a) == 0:
            return 0

        q_table = [] 

        for a in legal_a: 
            q_table.append(self.q_value(wrld, x, y))
            
        return max(q_table)

    def get_best_action(self, wrld):
        """return the best action given state-action pairs"""
        legal_a = self.get_legal_actions(wrld) 
        if len(legal_a) == 0:
            return 0

        # There could be more than one best action
        best_actions = list()

        for a in legal_a:
            new_wrld, events = wrld.next()
            new_q = self.q_value(new_wrld, self.x + a[0], self.y + a[1])
            qmax = self.get_qmax(new_wrld, self.x + a[0], self.y + a[1])
            if new_q >= qmax:
                best_actions.append(a)
        
        return best_actions

    def get_action(self, wrld):
        """take action with epsilon-greedy implementation"""
        legal_a = self.get_legal_actions(wrld)
        if len(legal_a) == 0:
            return 0 
        
        rand = random.random()
        if (rand < self.epsilon):
            new_action = random.choice(legal_a)
        else:
            best_actions = self.get_best_action(wrld)
            if len(best_actions) > 1: 
                new_action = random.choice(best_actions)
            else:
                new_action = best_actions[0]
        
        return new_action

    def calc_rewards(self, wrld, x, y):
        """
        Loop over heuristics function and evaluate at current worldstate
        return sum of heuristics
        """
        sum = 0
        if wrld.exit_at(x, y):
            sum += 200
        elif wrld.monsters_at(x,y) or wrld.explosion_at(x,y) or wrld.bomb_at(x,y):
            sum += -100
        return 1

    def update_weights(self, wrld, new_wrld, new_x, new_y):
        """update feature weights"""
        x, y = self.last_action[0], self.last_action[1]
        reward = self.calc_rewards(wrld, x, y)
        print("Reward: ", reward)
        current_q = self.q_value(wrld, x, y)
        next_action = self.get_action(wrld) 

        # delta = r + v(max(a')(Q(s',a'))) - Q(s,a)
        delta = (reward + (self.discount_factor * self.q_value(new_wrld, new_x, new_y))) - current_q

        fvec = self.extract_features(wrld, x, y)
        for f in fvec: 
            self.weights[f] = self.weights[f] + self.learning_rate  * delta * fvec[f]