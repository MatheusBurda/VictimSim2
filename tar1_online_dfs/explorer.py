# EXPLORER AGENT
# @Author: Tacla, UTFPR
#
### It walks randomly in the environment looking for victims. When half of the
### exploration has gone, the explorer goes back to the base.

import sys
import os
import random
import math
from abc import ABC, abstractmethod
from vs.abstract_agent import AbstAgent
from vs.constants import VS
from map import Map
import heapq

DEBUG = False

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]


class Stack:
    def __init__(self):
        self.items = []

    def push(self, item):
        self.items.append(item)

    def pop(self):
        if not self.is_empty():
            return self.items.pop()

    def is_empty(self):
        return len(self.items) == 0


class Explorer(AbstAgent):
    def __init__(self, env, config_file, resc=None, direction=0, resc_captain=None):
        """ Construtor do agente random on-line
        @param env: a reference to the environment 
        @param config_file: the absolute path to the explorer's config file
        @param resc: a reference to the rescuer agent to invoke when exploration finishes
        """

        super().__init__(env, config_file)
        self.walk_stack = Stack()  # a stack to store the movements
        self.untried_stack = Stack() # a stack to store the untried movements

        self.backtracking_stack = Stack()   # a stack to store the backtracking positions to a avaiable node
        self.time_to_come_back = False
        self.results = []           # a table to store results given results(previous_state, action) = state
        self.set_state(VS.ACTIVE)  # explorer is active since the beginning
        self.resc = resc           # reference to the rescuer agent
        self.x = 0                 # current x position relative to the origin 0
        self.y = 0                 # current y position relative to the origin 0
        self.map = Map()           # create a map for representing the environment
        self.victims = {}          # a dictionary of found victims: (seq): ((x,y), [<vs>])
                                   # the key is the seq number of the victim,(x,y) the position, <vs> the list of vital signals
        # put the current position - the base - in the map
        self.map.add((self.x, self.y), 1, VS.NO_VICTIM, self.check_walls_and_lim())
        
        # Added Atributes
        self.preferred_direction = direction
        self.cells_known = {(0,0): {"visited": True, "difficulty" : 1}}      # a table to store the visited cells
        self.resc_captain = resc_captain
        

    def get_next_position(self):
        """ Randomically, gets the next position that can be explored (no wall and inside the grid)
            There must be at least one CLEAR position in the neighborhood, otherwise it loops forever.
        """
        # Check the neighborhood walls and grid limits
        obstacles = self.check_walls_and_lim()
    
        # Loop until a CLEAR position is found
        while True:
            # Get a random direction
            direction = random.randint(0, 7)
            # Check if the corresponding position in walls_and_lim is CLEAR
            if obstacles[direction] == VS.CLEAR:
                return Explorer.AC_INCR[direction]
    

    def __get_current_pos(self) -> tuple:
        return (self.x, self.y)
    

    def actions(self) -> tuple:
        obstacles = self.check_walls_and_lim()

        possible_actions = []
        
        for i, obstacle in enumerate(obstacles):
            if obstacle == VS.CLEAR:
                action = Explorer.AC_INCR[i]
                possible_actions.append(action)
            else:
                possible_actions.append(None)
        
        # Rotate to preferred direction
        rotate_n = self.preferred_direction
        if rotate_n != 0:
            possible_actions = possible_actions[rotate_n:] + possible_actions[:rotate_n]

        possible_actions = [i for i in possible_actions if i is not None]

        return possible_actions


    def online_dfs(self):
        possible_actions = self.actions()

        current_pos = self.__get_current_pos()

        next_action = None

        for action in possible_actions:

            next_position = (current_pos[0] + action[0], current_pos[1] + action[1])
            
            if next_position not in self.cells_known.keys():
                self.cells_known[next_position] = {"visited": False, "difficulty" : None}

            if self.cells_known[next_position]["visited"] == False and not next_action:
                next_action = action

        if not next_action:
            return 0,0
        
        return next_action


    def get_adjacents_unvisited(self, location):
        adjacents = []
        for pos, key_value in self.cells_known.items():
            if abs(pos[0] - location[0]) <= 1 and abs(pos[1] - location[1]) <= 1 and (key_value["visited"] == False):
                adjacents.append(pos)

        return adjacents


    def backtrack(self):
        visited_locations = [key for key, value in self.cells_known.items() if value["visited"] == True]          

        possible_goals = []
        for pos in visited_locations:
            if len(self.get_adjacents_unvisited(pos)) > 0:
                possible_goals.append(pos)

        min_cost = None
        best_path = None
        for goal in possible_goals:
            path, cost = self.a_star_search(self.__get_current_pos(), goal)
            if path == [] or cost == -1:
                pass
            elif min_cost is None or cost < min_cost:
                min_cost = cost
                best_path = path

        if not best_path:
            return

        last_step = best_path[-1]
        for step in reversed(best_path[:-1]):
            delta_step = (step[0]-last_step[0], step[1]-last_step[1])
            self.backtracking_stack.push(delta_step)
            last_step = step


    def update_costs(self, current_point, next_point):
        dx = current_point[0] - next_point[0]
        dy = current_point[1] - next_point[1]
        
        difficulty = self.cells_known[next_point]["difficulty"]

        if dx == 0 or dy == 0:
            return difficulty * self.COST_LINE
        else:
            return difficulty * self.COST_DIAG


    def a_star_search(self, start, goal):

        def heuristic(a, b):
            (x1, y1) = a
            (x2, y2) = b
            return abs(x1 - x2) + abs(y1 - y2)

        def reconstruct_path(came_from, start, goal):
            current = goal
            path = []
            while current != start:
                path.append(current)
                current = came_from[current]
            path.append(start) 
            return path

        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        
        while not frontier.empty():
            current = frontier.get()
            
            if current == goal:
                break

            cells_nearby = []
            for pos, key_value in self.cells_known.items():
                if abs(pos[0] - current[0]) <= 1 and abs(pos[1] - current[1]) <= 1 and (key_value["visited"] == True or pos == goal):
                    cells_nearby.append(pos)

            for next in cells_nearby:
                new_cost = cost_so_far[current] + self.update_costs(current, next)
                if next not in cost_so_far.keys() or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current

        if goal not in came_from:
            return [], -1

        path = reconstruct_path(came_from, start, goal)

        return path, cost_so_far[goal]


    def explore(self):

        if not self.backtracking_stack.is_empty():
            dx, dy = self.backtracking_stack.pop()
        else:
            # get an random increment for x and y       
            dx, dy = self.online_dfs()

        if dx == dy == 0:
            self.backtrack()
            if not self.backtracking_stack.is_empty():
                dx, dy = self.backtracking_stack.pop()
            else:
                #TODO: return to the base (no more tiles unvisited)
                dx, dy = 0, 0

        # Moves the body to another position
        rtime_bef = self.get_rtime()
        result = self.walk(dx, dy)
        rtime_aft = self.get_rtime()

        # Test the result of the walk action
        # Should never bump, but for safe functionning let's test
        if result == VS.BUMPED:
            # update the map with the wall
            self.map.add((self.x + dx, self.y + dy), VS.OBST_WALL, VS.NO_VICTIM, self.check_walls_and_lim())

        if result == VS.EXECUTED:
            # check for victim returns -1 if there is no victim or the sequential
            # the sequential number of a found victim
            self.walk_stack.push((dx, dy))

            # update the agent's position relative to the origin
            self.x += dx
            self.y += dy    

            self.cells_known[self.__get_current_pos()]["visited"] = True

            # Check for victims
            seq = self.check_for_victim()
            # has_read_vital = False
            if seq != VS.NO_VICTIM and (self.__get_current_pos() not in self.victims.keys()):
                vs = self.read_vital_signals()
                # has_read_vital = True
                self.victims[self.__get_current_pos()] = {'id': seq, 'signals' : vs}
                print(f"{self.NAME} Victim found at ({self.x}, {self.y}), rtime: {self.get_rtime()}")
            
            # Calculates the difficulty of the visited cell
            difficulty = (rtime_bef - rtime_aft)
            if dx == 0 or dy == 0:
                difficulty = difficulty / self.COST_LINE
            else:
                difficulty = difficulty / self.COST_DIAG
            
            self.cells_known[self.__get_current_pos()]["difficulty"] = difficulty

            # Update the map with the new cell
            self.map.add((self.x, self.y), difficulty, seq, self.check_walls_and_lim())

        return
    

    def stack_comeback(self, path):
        if not self.walk_stack.is_empty():
            self.walk_stack = Stack()
        while len(path) > 1:
            dx = path[1][0] - path[0][0]
            dy = path[1][1] - path[0][1]
            self.walk_stack.push((dx, dy))
            path.pop(0)


    def come_back(self):
        # Will keep executing untill stack is empty
        dx, dy = self.walk_stack.pop()
        dx = dx * -1
        dy = dy * -1

        result = self.walk(dx, dy)
        if result == VS.BUMPED:
            print(f"{self.NAME}: when coming back bumped at ({self.x+dx}, {self.y+dy}) , rtime: {self.get_rtime()}")
            return
        
        if result == VS.EXECUTED:
            # update the agent's position relative to the origin
            self.x += dx
            self.y += dy
        

    def deliberate(self) -> bool:
        """ The agent chooses the next action. The simulator calls this
        method at each cycle. Must be implemented in every agent"""

        if self.time_to_come_back and not self.walk_stack.is_empty():
            self.come_back()

        else:
            path, cost = self.a_star_search(self.__get_current_pos(), (0,0))

            error = self.COST_DIAG + self.COST_FIRST_AID + self.COST_READ
            if cost + error < self.get_rtime():
                self.explore()
                return True
            elif self.time_to_come_back == False:
                print(f"{self.NAME}: RETURNING TO BASE ##########")
                self.stack_comeback(path)
                self.time_to_come_back = True

        # time to come back to the base
        if self.walk_stack.is_empty() or (self.x == 0 and self.y == 0):

            print(f"{self.NAME}: rtime {self.get_rtime()}, sending map")

            self.resc_captain.cap_receive_map(self.cells_known, self.victims)
            return False

        return True

