# This is necessary to find the main code
import math
import sys
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
        self.learning_rate = 0.3
        self.discount_factor = 0.8
        self.epsilon = 0.3
        self.weights = weights
        self.last_q = 0
        self.current_action = (0,0)
        self.current_pos = (0,0)
        self.last_pos = (0,0)
    
    def do(self, wrld):
        # Find character
        c = wrld.me(self)

        if self.last_q:
            self.update_weights(wrld, c)

        # Place bomb if monster or walls are nearby
        if qf.monster_within_radius(wrld, c.x, c.y) or qf.find_walls(wrld, c.x, c.y):
            self.place_bomb()

        # print("\n========================")
        # print("MOVING CHARACTER: ")
        # # Move character according to qlearning
        # print(" q: ", self.last_q)
        move = self.get_action(wrld, c.x, c.y)
        self.current_action = move
        self.last_q = self.q_value(wrld, self.current_action, c.x, c.y)
        self.last_pos = (c.x, c.y)
        self.current_pos = (c.x + move[0], c.y + move[1])

        self.move(move[0], move[1])

    def get_legal_actions(self, wrld, curr_pos):
        """Returns all possible actions given a position"""
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
                        if not wrld.wall_at(x + c_dx, y + c_dy) and not wrld.bomb_at(x + c_dx, y + c_dy) and not wrld.explosion_at(x + c_dx, y + c_dy):
                            # Set move in Sensed World
                            directions.append((c_dx, c_dy))
        
        return directions

    def extract_features(self, wrld, x, y):
        """Returns a dictionary of calculated features"""
        features = {}
        features['dist_to_monsters'] = qf.distance_to_monster(wrld, x, y)
        features['dist_to_exit'] = qf.distance_to_exit(wrld, x, y)
        features['bomb_range'] = qf.bomb_radius(wrld, x, y)
        features['blast'] = qf.if_expl(wrld, x, y)

        return features

    def q_value(self, wrld, action, x, y):
        """Finds the qvalue of a state-action pair"""
        q = 0 
        fvec = self.extract_features(wrld, x + action[0], y + action[1]) 
        # print("FVEC: ", fvec)
        for f in fvec: 
            q += self.weights[f] * fvec[f] 
        return q

    def get_best_action(self,wrld,x,y):
        """
        Calculate the best action for character taking into account monster
        makes optimal moves with limited visibility.
        Returns best action and Q(s,a) as a tuple 
        """

        best_action = (0,0)
        qmax = 0
        m = None
        q_table = {}

        m = qf.find_closest_monster(wrld, x, y)
        
        #Iterate through possible character moves
        legal_a = self.get_legal_actions(wrld,(x,y))
        for a in legal_a:

            # If character is still alive
            if wrld.me(self):
                wrld.me(self).move(a[0], a[1])

                # If there is a monster
                if m:
                    # Assume monster has limited visibility
                    m_loc = (m.x, m.y)
                    m_best_step = (0,0)

                    # Find optimal monster move
                    m_moves = self.get_legal_actions(wrld, m_loc)
                    path = qf.astar(m_loc, (x + a[0], y + a[1]), wrld)

                    
                    if len(path) > 1:
                        next_pos = path[1]
                        m_best_step = (next_pos[0] - m.x, next_pos[1] - m.y)
                    # for m_move in m_moves: 
                    #     m_new_loc = (m.x + m_move[0], m.y + m_move[1])
                    #     path = qf.astar(m_new_loc, (x + a[0], y + a[1]), wrld)
                    #     if len(path) < m_best_path:
                    #         m_best_step = m_move 
                    #         m_best_path = len(path)
                    
                    # Set monster move in Sensed World
                    m.move(m_best_step[0], m_best_step[1])
                    # Go to next state
                    next_state, events = wrld.next()
                
                    # Find optimal character move assuming monster makes best move for self
                    q = self.q_value(next_state, a, x, y)
                    q_table[q] = a
        
        qtable = list(q_table.keys())
        if len(qtable) > 0:
            qmax = max(qtable)
            best_action = q_table[qmax]
        
        return best_action, qmax

    def get_action(self, wrld, x, y):
        """Take action with epsilon-greedy implementation"""
        new_action = (0,0)

        legal_a = self.get_legal_actions(wrld, (x,y))

        # Generate random number
        r = random.random()
        # print("Rando: ", r)
        if len(legal_a) > 0:
            if (r < self.epsilon):
                new_action = random.choice(legal_a)
            else:
                new_action = self.get_best_action(wrld, x, y)[0]

        return new_action 
    
    def eval_state(self, wrld, curr_pos):
        # If no events,  evaluate state based on ratio between distance to monster and exit
        x, y = curr_pos[0], curr_pos[1]
        
        exit_loc = qf.find_exit(wrld) 
        exit_path = qf.astar((x,y), exit_loc, wrld)
        exit_length = len(exit_path) 

        m = qf.find_closest_monster(wrld, x, y)

        if not m: # If there is no monster
            return 0 

        m_path = qf.astar((x,y), (m.x, m.y), wrld)
        m_length = len(m_path) 

        # mdist = qf.distance_to_monster(wrld, curr_x, curr_y)
        # edist = qf.distance_to_exit(wrld, curr_x, curr_y)

        return (1/(exit_length + 1)) - (1/(m_length + 1))

    def calc_rewards(self, wrld, x, y):
        """
        Loop over heuristics function and evaluate at current worldstate
        return sum of heuristics
        """
        if wrld.exit_at(x,y):
            r = 100
        elif wrld.bomb_at(x,y) or wrld.explosion_at(x,y) or wrld.monsters_at(x,y):
            r = -50
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
        """Update the weights for Q(s,a)"""
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