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

# Main Q Learning Agent 
class QAgent(CharacterEntity):
    def __init__(self, name, player, x, y, weights):
        CharacterEntity.__init__(self, name, player, x, y) 
        self.learning_rate = 0.2
        self.discount_factor = 0.9
        self.epsilon = 0.2
        self.last_q = 0
        self.current_action = (0,0)
        self.weights = weights
        self.current_pos = (0,0)
        self.last_pos = (0,0)
        
    def do(self, wrld):
         
        c = wrld.me(self)

        if self.last_q:
            self.update_weights(wrld, c)

        # Place bomb if monster or walls are nearby
        if qf.monster_within_radius(wrld, c.x, c.y) or qf.find_walls(wrld, c.x, c.y):
            self.place_bomb()

        # print("\n========================")
        # print("MOVING CHARACTER: ")
        # Move character according to qlearning
        # print(" q: ", self.last_q)
        move = self.get_action(wrld, c.x, c.y)
        self.current_action = move
        self.last_q = self.q_value(wrld, self.current_action, c.x, c.y)
        self.last_pos = (c.x, c.y)
        self.current_pos = (c.x + move[0], c.y + move[1])

        self.move(move[0], move[1])

        # print("Last pos: ", self.last_pos[0], self.last_pos[1])
        # print("Current action:", self.current_action)
        # print("last q: ", self.last_q)
        # print("Current pos: ", self.current_pos[0], self.current_pos[1])

    def get_legal_actions(self, wrld, curr_pos):
        """get possible actions given current position"""
        x, y = (curr_pos[0], curr_pos[1])
        directions = []

        for c_dx in [-1,0,1]:
            # Avoid out-of-bound indexing 
            if (x + c_dx >= 0) and (x + c_dx < wrld.width()):
                # Loop through y directions
                for c_dy in [-1,0,1]:
                    # Avoid out-of-bound indexing
                    if (y + c_dy >= 0) and (y + c_dy < wrld.height()):
                        # No need to check impossible moves
                        if not wrld.wall_at(x + c_dx, y + c_dy) or not wrld.bomb_at(x + c_dx, y + c_dy):
                            # Set move in Sensed World
                            directions.append((c_dx, c_dy))
        
        return directions
    
    def extract_features(self, wrld, x, y):
        """get list of feature values"""
        features = {}
        features['dist_to_monsters'] = qf.distance_to_monster(wrld, x, y)
        features['dist_to_exit'] = qf.distance_to_exit(wrld, x, y)
        features['dist_to_bomb'] = qf.distance_to_bomb(wrld, x, y)
        features['blast'] = qf.blast_radius(wrld, x, y)
        # features['blast'] = qf.bomb_radius(wrld, x, y)
        # features['monster_nearby'] = qf.monster_within_radius(wrld,x,y)
        # moves = self.get_legal_actions(wrld, (x, y))
        # features['cornered'] = float(len(moves) < 3)
        return features

    def q_value(self, wrld, action, x, y):
        """find the qvalue of a state-action pair"""
        q = 0 
        fvec = self.extract_features(wrld, x + action[0], y + action[1]) 
        # print("FVEC: ", fvec)
        for f in fvec: 
            q += self.weights[f] * fvec[f] 
        return q

    def get_best_action(self,wrld,x,y):
        """return best action given all state-action pairs"""

        best_action = (0,0)
        qmax = 0
        m = None
        q_table = {}


        # Get position of all monsters
        monsters = wrld.monsters.values()
        if len(monsters) == 1:
            m = next(iter(monsters))[0]
        elif len(monsters) == 2:
            monster = iter(wrld.monsters.values())
            m1 = next(monster)[0]
            m2 = next(monster)[0]
            m_pos_list = [(m1.x, m1.y), (m2.x, m2.y)]
            m_closest = qf.find_closest_obj(wrld, m_pos_list, x, y)
            if m_closest == m_pos_list[0]:
                m = m1
            else:
                m = m2 

        # Iterate through possible character moves
        legal_a = self.get_legal_actions(wrld, (x,y))
        for a in legal_a:
            if wrld.me(self):
                wrld.me(self).move(a[0],a[1])

                if m is not None:
                    # Determine closest monster
                    m_loc = (m.x, m.y)
                    m_best_path = wrld.height() # Arbitrary value
                    m_best_step = (0,0)

                    # Find optimal monster move
                    for m_dx in [-1,0,1]:
                        if (m.x + m_dx >= 0) and (m.x + m_dx < wrld.width()):
                            for m_dy in [-1,0,1]:
                                if (m_dx != 0) or (m_dy != 0):
                                    if (m.y + m_dy >= 0) and (m.y + m_dy < wrld.height()):
                                        if not wrld.wall_at(m.x + m_dx, m.y + m_dy) and not qf.distance_to_bomb(wrld, m.x + m_dx, m.y + m_dy):
                                                m_new_loc = (m.x + m_dx, m.y + m_dy)
                                                path = qf.astar(m_new_loc, (x + a[0], y + a[1]), wrld)
                                                if len(path) < m_best_path:
                                                    m_best_step = (m_dx, m_dy)
                                                    m_best_path = len(path)
                                                            
                    # Set monster move in Sensed World
                    # print("Monster best step: ", m_best_step)
                    m.move(m_best_step[0], m_best_step[1])
                
                    m_new_loc = (m.x + m_best_step[0], m.y + m_best_step[1])
                # Go to next state
                next_state, events = wrld.next()
                
                # Find optimal character move assuming monster makes best move for self
                q = self.q_value(next_state, a, x, y)
                q_table[q] = a
        
        qtable = list(q_table.keys())
        if len(qtable) > 0:
            qmax = max(qtable)
            best_action = q_table[qmax]
        
        # print("TABLE: ", q_table)
        # print("QMAX: ", qmax, "FROM ACTION", best_action)
        # print("Final new character location:", c.x + best_action[0], c.y + best_action[1])
        return best_action, qmax

    def get_action(self, wrld, x, y):
        """take action with epsilon-greedy implementation"""
        new_action = (0,0)

        legal_a = self.get_legal_actions(wrld, (x,y))

        # Generate random number
        r = random.random()
        # print("Rando: ", r)
        if (r < self.epsilon):
            new_action = random.choice(legal_a)
        else:
            new_action = self.get_best_action(wrld, x, y)[0]

        return new_action 
    
    def eval_state(self, wrld, curr_pos):
        # If no events,  evaluate state based on ratio between distance to monster and exit
        curr_x, curr_y = curr_pos[0], curr_pos[1]
        mdist = qf.distance_to_monster(wrld, curr_x, curr_y)
        edist = qf.distance_to_exit(wrld, curr_x, curr_y)

        return edist - mdist

    def calc_rewards(self, wrld, x, y):
        """
        Loop over heuristics function and evaluate at current worldstate
        return sum of heuristics
        """
        r = 0
        if wrld.exit_at(x,y):
            r = 50
        elif wrld.bomb_at(x,y) or wrld.explosion_at(x,y) or wrld.monsters_at(x,y):
            r = -25
        else:
            r = self.eval_state(wrld, (self.current_pos[0], self.current_pos[1]))
        return r

    def next_best_state(self, current_state, x, y, a):
        """Get the next best state's q value from a given position"""
        next_pos = (x + a[0], y + a[1])  # Where character is located in next state
        current_state.me(self).move(a[0],a[1]) # Move character in the sensed world 
        next_state, events = current_state.next()
        next_action, q = self.get_best_action(next_state, next_pos[0], next_pos[1]) # Get best move in this state  

        return q
        
    def update_weights(self, current_state, c):
        """update the weights for Q(s,a)"""
        # print("\n===========================")
        # print("UPDATING WEIGHTS:")
        current_action = self.current_action
        # print("Current action: ", current_action)
        reward = self.calc_rewards(current_state, c.x, c.y)
        # print(current_state.monsters_at(self.current_pos[0],self.current_pos[1]))
        current_utility = self.q_value(current_state, current_action, c.x, c.y)

        # Get best action in next state
        q = self.next_best_state(current_state, c.x, c.y, current_action)
       
        # print("Reward;", reward, "Current utility:", current_utility)

        # delta = reward + v(max(a')(Q(s',a'))) - Q(s,a)
        delta = (reward + (self.discount_factor * q)) - current_utility
        
        # w = w + alpha * delta * f(s,a)
        fvec = self.extract_features(current_state, c.x, c.y)
        for f in fvec: 
            self.weights[f] = self.weights[f] + self.learning_rate  * delta * fvec[f]

    def end_calc_reward(self, wrld, x, y):
        if wrld.exit_at(x,y):
            return 50
        return -25

    def done(self, current_state):
        """if character has died, calculate final reward"""
        # print("\n===========================")
        # print("FINAL WEIGHTS UPDATE")
        
        current_action = self.current_action
        reward = self.end_calc_reward(current_state, self.current_pos[0], self.current_pos[1])
        best_q = 0

        # Get best action in next state
        legal_a = self.get_legal_actions(current_state, self.last_pos)
        for a in legal_a:
            new_q = self.q_value(current_state, a, self.last_pos[0], self.last_pos[1])
            if new_q > best_q:
                best_q = new_q

        # delta = reward + v(max(a')(Q(s',a'))) - Q(s,a)
        delta = (reward + (self.discount_factor * best_q)) - self.last_q
        
        # w = w + alpha * delta * f(s,a)
        fvec = self.extract_features(current_state, self.last_pos[0], self.last_pos[1])
        for f in fvec: 
            self.weights[f] = self.weights[f] + self.learning_rate  * delta * fvec[f]