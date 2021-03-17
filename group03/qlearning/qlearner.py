# This is necessary to find the main code
import math
import sys
import random 

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
from sensed_world import SensedWorld
from events import Event 

sys.path.insert(1, '../bomberman/group03')
import qfunctions as qf
from actions import Actions, Pos

# Main Q Learning Agent 
class QAgent(CharacterEntity):
    def __init__(self, name, player, x, y, weights):
        CharacterEntity.__init__(self, name, player, x, y) 
        self.learning_rate = 0.01
        self.discount_factor = 0.7
        self.epsilon = 0.25
        self.weights = weights
        self.last_q = 0
        self.current_action = (0,0)
        self.current_pos = (0,0)
        self.last_pos = (0,0)
        self.last_c = None
        self.win = 0

    def do(self, wrld):
        # Find character
        c = wrld.me(self)

        if self.last_q:
            last_action = self.current_action
            self.update_weights(wrld, last_action, self.last_c)

        action = self.get_action(wrld, c.x, c.y)
        move = Pos[action].value

        self.current_action = move
        self.last_q = self.q_value(wrld, self.current_action, c.x, c.y)
        self.last_pos = (c.x, c.y)
        self.last_c = c
        self.current_pos = (c.x + move[0], c.y + move[1])

        # Place bomb if bomb action is selected
        if action == "BOMB":
            self.place_bomb()
        print('\n')
        print('PICKED')
        print(c.x + move[0], c.y + move[1])
        self.move(move[0], move[1])


    def get_legal_actions(self, wrld, curr_pos):
        """Returns all possible actions given a position"""
        x, y = (curr_pos[0], curr_pos[1])
        actions = []

        for i in range(10):
            a = Actions(i).name
            dx = Pos[a].value[0]
            dy = Pos[a].value[1]
            if (x + dx >= 0) and (x + dx < wrld.width()):
                if (y + dy >= 0) and (y + dy < wrld.height()):
                    if not wrld.wall_at(x + dx, y + dy) and not wrld.bomb_at(x + dx, y + dy) and not wrld.explosion_at(x + dx, y + dy):
                        if a == "BOMB" and len(wrld.bombs) > 0:
                            continue
                        actions.append(a)
        return actions

    def extract_features(self, wrld, x, y):
        """Returns a dictionary of calculated features"""
        features = {}
        features['dist_to_monsters'] = qf.distance_to_monster(wrld, x, y)
        features['dist_to_exit'] = qf.distance_to_exit(wrld, x, y)
        features['bomb_range'] = qf.bomb_radius(wrld, x, y)
        features['blast'] = qf.if_expl(wrld, x, y)
        features['blocked'] = qf.if_blocked(wrld,x,y)
        # features['m_range'] = qf.monster_within_radius(wrld,x,y)
        return features

    def q_value(self, wrld, action, x, y):
        """Finds the qvalue of a state-action pair"""
        print("X:", x + action[0]," Y:", y + action[1])
        q = 0 
        fvec = self.extract_features(wrld, x + action[0], y + action[1]) 
        #print("FVEC: ", fvec)
        for f in fvec:
            print(f, self.weights[f], fvec[f])
            q += self.weights[f] * fvec[f] 
        print(q)
        return q

    def get_best_action(self,wrld,x,y):
        """
        Calculate the best action for character taking into account monster
        makes optimal moves with limited visibility.
        Returns best action and Q(s,a) as a tuple 
        """
        best_action = "STAY"
        qmax = 0
        m = qf.find_closest_monster(wrld, x, y)
        q_table = {}
        
        #Iterate through possible character moves
        legal_a = self.get_legal_actions(wrld,(x,y))
        print(legal_a)

        for action in legal_a:
            # If character is still alive
            if wrld.me(self):
                a = Pos[action].value
                # If we want to place a bomb
                if action == "BOMB":
                    wrld.me(self).place_bomb()
                # Move character
                wrld.me(self).move(a[0], a[1])
                print(a[0], a[1])

                # If there is a monster
                if m:
                    # Assume monster has limited visibility
                    m_loc = (m.x, m.y)
                    m_best_step = (0,0)

                    # Find optimal monster move
                    path = qf.astar(m_loc, (x + a[0], y + a[1]), wrld)
                    
                    if len(path) > 1:
                        next_pos = path[1]
                        m_best_step = (next_pos[0] - m.x, next_pos[1] - m.y)
                    
                    # Set monster move in Sensed World
                    m.move(m_best_step[0], m_best_step[1])
                    # Go to next state
                    next_state, events = wrld.next()
                    next_state.printit()
                    # Find optimal character move assuming monster makes best move for self
                    q = self.q_value(next_state, a, x, y)
                    q_table[action] = q
        
        qtable = list(q_table.values())
        if len(qtable) > 0:
            qmax = max(qtable)
            for k,v in q_table.items():
                if v == qmax:
                    best_action = k

        return best_action, qmax

    def get_action(self, wrld, x, y):
        """Take action with epsilon-greedy implementation"""
        new_action = "STAY"

        legal_a = self.get_legal_actions(wrld, (x,y))
        # Generate random number
        r = random.random()

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

        return (1/(exit_length + 1)) - (1/(m_length + 1))

    def calc_rewards(self, wrld, x, y):
        """
        Loop over heuristics function and evaluate at current worldstate
        return sum of heuristics
        """
        r = 0
        if wrld.exit_at(x,y):
            r = 150
        elif wrld.bomb_at(x,y) or wrld.explosion_at(x,y) or wrld.monsters_at(x,y):
            r = 50
        elif len(wrld.events) > 0:
            for e in wrld.events:
                if e.tpe == Event.BOMB_HIT_MONSTER:
                    r = 20
                elif e.tpe == Event.BOMB_HIT_WALL and wrld.me(self) is not None:
                    r = 20
        else:
            r = 1
        return r

    def next_best_state(self, current_state, x, y, a):
        """Get the next best state's q value from a given position"""
        next_pos = (x + a[0], y + a[1])  # Where character is located in next state
        current_state.me(self).move(a[0],a[1]) # Move character in the sensed world 
        next_state, events = current_state.next()
        next_action, q = self.get_best_action(next_state, next_pos[0], next_pos[1]) # Get best move in this state  

        return q
        
    def update_weights(self, current_state, current_action, c):
        """Update the weights for Q(s,a)"""
        print("\n===========================")
        # print("UPDATING WEIGHTS:")
        print("Current action: ", current_action)
        reward = self.calc_rewards(current_state, c.x, c.y)
        #print(current_state.monsters_at(self.current_pos[0],self.current_pos[1]))
        current_utility = self.q_value(current_state, current_action, c.x, c.y)

        # Get best action in next state
        q = self.next_best_state(current_state, c.x, c.y, current_action)

        #print("Reward:", reward, "Current utility:", current_utility)

        # delta = reward + v(max(a')(Q(s',a'))) - Q(s,a)
        delta = (reward + (self.discount_factor * q)) - current_utility
        
        # w = w + alpha * delta * f(s,a)
        fvec = self.extract_features(current_state, c.x, c.y)
        for f in fvec:
            self.weights[f] = self.weights[f] + self.learning_rate  * delta * fvec[f]