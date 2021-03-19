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
        self.learning_rate = 0.25
        self.discount_factor = 0.75
        self.epsilon = 0.15
        self.weights = weights
        self.last_q = 0
        self.current_action = (0,0)
        self.current_pos = (0,0)
        self.last_pos = (0,0)
        self.bombMove = (0,0)

    def do(self, wrld):
        # Find character
        c = wrld.me(self)
        if self.last_q:
            self.update_weights(wrld, c)

        #check if astar would be optimal instead
        astarAct = self.get_astar_action(wrld, c.x, c.y)
        #astarAct = None
        if astarAct:
            action = astarAct

        #else try to get best move based on q_value
        else:
            action = self.get_action(wrld, c.x, c.y)

        move = Pos[action].value

        self.current_action = move
        self.last_q = self.q_value(wrld, self.current_action, c.x, c.y)
        self.last_pos = (c.x, c.y)
        self.current_pos = (c.x + move[0], c.y + move[1])
        # print("======================")
        # print("LAST POS: ", self.last_pos)
        # print("======================")
        # Place bomb if bomb action is selected
        if action == "BOMB":
            self.place_bomb()
            move = self.bombMove

        self.move(move[0], move[1])
        #self.update_weights(wrld, c)


    def get_legal_actions(self, wrld, curr_pos):
        """Returns all possible actions given a position"""
        x, y = (curr_pos[0], curr_pos[1])
        actions = []

        for i in range(10):
            a = Actions(i).name
            dx = Pos[a].value[0]
            dy = Pos[a].value[1]

            if a == "BOMB": # add bomb as an action
                if len(wrld.bombs) > 0 or len(wrld.explosions) > 0:
                    continue
                else:
                    actions.append(a)
                    continue

            if (x + dx >= 0) and (x + dx < wrld.width()):
                if (y + dy >= 0) and (y + dy < wrld.height()):
                    if not wrld.wall_at(x + dx, y + dy) and not wrld.bomb_at(x + dx, y + dy) and not wrld.explosion_at(x + dx, y + dy):
                        if a == "BOMB" and len(wrld.bombs) > 0:
                            continue
                        actions.append(a)
        return actions

    def extract_features(self, wrld, x, y, isBomb):
        """Returns a dictionary of calculated features"""
        features = {}
        features['dist_to_monsters'] = qf.distance_to_monster(wrld, x, y)
        features['dist_to_exit'] = qf.distance_to_exit(wrld, x, y)
        features['bomb_range'] = qf.bomb_radius(wrld, x, y)

        features['m_to_bomb'] = qf.m_to_bomb(wrld, x, y)
        features['if_bomb_w'] = qf.if_bomb_wall(wrld, x, y)
        # if isBomb:
        #     features['m_to_bomb'] = qf.m_to_bomb(wrld,x,y)
        #     features['if_bomb_w'] = qf.if_bomb_wall(wrld, x, y)
        # else:
        #     features['m_to_bomb'] = 0
        #     features['if_bomb_w'] = 0

        # features['if_blocked'] = qf.if_blocked(wrld,x,y)
        return features

    def q_value(self, wrld, action, x, y):
        """Finds the qvalue of a state-action pair"""
        q = 0
        isBomb = False
        if action == (2,2):
            action = (0,0)
            isBomb = True
        fvec = self.extract_features(wrld, x + action[0], y + action[1], isBomb)
        # print("FOR ACTION: ", action)
        for f in fvec: 
            # print("FEATURE: ", f, "| VALUE: ", fvec[f], "| WEIGHT:  ", self.weights[f])
            q += self.weights[f] * fvec[f]
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
        
        for action in legal_a:
            # If character is still alive
            if wrld.me(self):
                a = Pos[action].value
                # If we want to place a bomb
                if action == "BOMB":
                    wrld.me(self).place_bomb()
                    next_state, events = wrld.next()
                    # # # Find optimal character move assuming monster makes best move for self
                    best_act= self.get_best_action(next_state, x, y)
                    # # #q = self.q_value(next_state, a, x, y)
                    # a = best_act[0]
                    self.bombMove = Pos[best_act[0]].value
                    next_state.me(self).move(self.bombMove[0], self.bombMove[1])
                    next, events = next_state.next()

                    q_table[action] = best_act[1]
                    continue # no movement needed
                # Move character
                if not (action == "BOMB"):
                    wrld.me(self).move(a[0], a[1])

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
                    
                    # print("MONSTER BEST POSITION: ", next_pos)
                    # Set monster move in Sensed World
                    m.move(m_best_step[0], m_best_step[1])
                    # Go to next state
                    # next_state, events = wrld.next()
                    # # Find optimal character move assuming monster makes best move for self
                    # q = self.q_value(next_state, a, x, y)
                    # q_table[action] = q

                next_state, events = wrld.next()
                # Find optimal character move assuming monster makes best move for self
                q = self.q_value(next_state, a, x, y)
                q_table[action] = q
        
        # print("QTABLE: ", q_table)
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
                best_act = self.get_best_action(wrld, x, y)
                new_action = best_act[0]
                new_pos = best_act[1]

        return new_action

    def get_astar_action(self, wrld, x, y):
        """Get the next action on the astar path if there is an exit that is not blocked and no monster"""
        exit = qf.find_exit(wrld)
        action = "STAY"
        if exit:
            path = qf.astar((x,y), exit, wrld)
            if len(path) > 1 and not qf.find_closest_monster(wrld, x, y):
                action = Pos((path[1][0] - x, path[1][1] - y)).name
                return action
        return None

    def calc_rewards(self, wrld, pos, action):
        """
        Loop over heuristics function and evaluate at current worldstate
        return sum of heuristics
        """
        # x, y = (pos[0] + action[0], pos[1] + action[1])
        # next_state, _ = wrld.next()
        #
        # # If character can exit
        # if next_state.exit_at(x,y):
        #     return 60
        # # If character will died due to explosion or monster
        # elif next_state.explosion_at(x,y) or wrld.monsters_at(x,y):
        #     return -25
        # # If there is a bomb but it hasn't gone off yet
        # elif next_state.bomb_at(x,y):
        #     return -15
        # else:
        #     return -1 # Cost of living
        x = pos[0] + action[0]
        y = pos[1] + action[1]
        r = 0
        if wrld.exit_at(x, y):
            return 100
        # elif wrld.bomb_at(x, y) or wrld.explosion_at(x, y) or wrld.monsters_at(x, y):
        #     return -60
        elif len(wrld.events) > 0:
            for e in wrld.events:
                if e.tpe == Event.BOMB_HIT_MONSTER:  # and not not wrld.me(self):
                    r += 40
                if e.tpe == Event.BOMB_HIT_WALL:  # and not not wrld.me(self):
                    r += 5
                if e.tpe == Event.CHARACTER_KILLED_BY_MONSTER:
                    print ("hi")
                    return -60
                if e.tpe == Event.CHARACTER_FOUND_EXIT:
                    print("exit")
                    return 100
                if (action == "BOMB" and r == 0): #for wasting bomb
                    r -= 5
        else:
            r -= 1
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
        if not c:
            currPos = self.last_pos
        else:
            currPos = (c.x, c.y)
        reward = self.calc_rewards(current_state, currPos, current_action)
        # print(current_state.monsters_at(self.current_pos[0],self.current_pos[1]))
        current_utility = self.q_value(current_state, current_action, currPos[0], currPos[1])

        # Get best action in next state
        if not c:
            q = 0 #game is done
        else:
            q = self.next_best_state(current_state, currPos[0],currPos[1], current_action)

        print("Reward;", reward, "Current utility:", current_utility)

        # delta = reward + v(max(a')(Q(s',a'))) - Q(s,a)
        delta = (reward + (self.discount_factor * q)) - current_utility
        isBomb = False
        if current_action == "BOMB":
            isBomb = True
        # w = w + alpha * delta * f(s,a)
        fvec = self.extract_features(current_state, currPos[0], currPos[1], isBomb)
        for f in fvec: 
            self.weights[f] = self.weights[f] + self.learning_rate  * delta * fvec[f]

