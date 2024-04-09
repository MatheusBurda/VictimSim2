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
    def __init__(self, env, config_file, resc):
        """ Construtor do agente random on-line
        @param env: a reference to the environment 
        @param config_file: the absolute path to the explorer's config file
        @param resc: a reference to the rescuer agent to invoke when exploration finishes
        """

        super().__init__(env, config_file)
        self.walk_stack = Stack()  # a stack to store the movements
        #TODO: is it used?
        self.untried_stack = Stack() # a stack to store the untried movements

        self.backtracking_stack = Stack()   # a stack to store the backtracking positions to a avaiable node
        self.results = []           # a table to store results given results(previous_state, action) = state
        self.cells_known = {(0,0): {"visited": True, "cost_to_origin" : 0}}      # a table to store the visited cells
        self.set_state(VS.ACTIVE)  # explorer is active since the beginning
        self.resc = resc           # reference to the rescuer agent
        self.x = 0                 # current x position relative to the origin 0
        self.y = 0                 # current y position relative to the origin 0
        self.map = Map()           # create a map for representing the environment
        self.victims = {}          # a dictionary of found victims: (seq): ((x,y), [<vs>])
                                   # the key is the seq number of the victim,(x,y) the position, <vs> the list of vital signals

        # put the current position - the base - in the map
        self.map.add((self.x, self.y), 1, VS.NO_VICTIM, self.check_walls_and_lim())


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
        
        # self.cells_known[self.__get_current_pos()]["obstacles"] = obstacles

        #TODO: Row to preferred direction

        possible_actions = []
        
        for i, obstacle in enumerate(obstacles):

            if obstacle == VS.CLEAR:
                action = Explorer.AC_INCR[i]
                possible_actions.append(action)
        
        return possible_actions


    def online_dfs(self):
        
        possible_actions = self.actions()

        current_pos = self.__get_current_pos()

        next_action = None

        for action in possible_actions:

            next_position = (current_pos[0] + action[0], current_pos[1] + action[1])
            
            if next_position not in self.cells_known.keys():
                self.cells_known[next_position] = {"visited": False}

            if self.cells_known[next_position]["visited"] == False and not next_action:
                next_action = action

        if not next_action:
            return 0,0
        
        return next_action


    def backtrack(self):
        # TODO: Implement A* to find best way out to a valid 
        possible_goals = [key for key, value in self.cells_known.items() if value["visited"] == False]
        
        min_cost = None
        best_path = None
        print(self.cells_known.items())
        print(possible_goals)
        for goal in possible_goals:
            print(self.__get_current_pos(), goal)
            path, cost = self.a_star_search(self.__get_current_pos(), goal)
            if min_cost is None or cost < min_cost:
                min_cost = cost
                best_path = path

        last_step = best_path[-1]
        for step in reversed(best_path[:-1]):
            delta_step = (last_step[0]-step[0], last_step[1]-step[1])
            self.backtracking_stack.push(delta_step)


    def a_star_search(self, start, goal):

        def heuristic(a, b) -> float:
            (x1, y1) = a
            (x2, y2) = b
            return abs(x1 - x2) + abs(y1 - y2)

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

            cells_nearby = [pos for pos, _ in self.cells_known.items() if abs(pos[0] - current[0]) == 1 or abs(pos[1] - current[1]) == 1]

            for next in cells_nearby:
                new_cost = cost_so_far[current] + self.update_costs(current, next)
                if next not in cost_so_far.keys() or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current
        
        return came_from, cost_so_far[came_from[-1]]


    def update_costs(self, current_point, next_point):
        dx = current_point[0] - next_point[0]
        dy = current_point[1] - next_point[1]
        
        if abs(dx) > 0 and abs(dy) > 0:
            return self.COST_DIAG
        elif abs(dx) > 0 or abs(dy) > 0:
            return self.COST_LINE
        return 0


    def explore(self):

        if not self.backtracking_stack.is_empty():
            dx, dy = self.backtracking_stack.pop()
        else:
            # get an random increment for x and y       
            dx, dy = self.online_dfs()

        if dx == dy == 0:
            self.backtrack()
            if len(self.backtracking_stack) > 0:
                dx, dy = self.backtracking_stack.pop()
            else:
                #TODO: return to the base (no more tiles unvisited)
                dx, dy = 0, 0

        # print(self.cells_known, self.__get_current_pos())

        # new_cost = self.cells_known[self.__get_current_pos()]

        # if abs(dx) > 0 and abs(dx) > 0:
        #     new_cost += self.COST_DIAG
        # elif abs(dx) > 0 or abs(dx) > 0:
        #     new_cost += self.COST_LINE

        # Moves the body to another position
        rtime_bef = self.get_rtime()
        result = self.walk(dx, dy)
        rtime_aft = self.get_rtime()

        # Test the result of the walk action
        # Should never bump, but for safe functionning let's test
        if result == VS.BUMPED:
            # update the map with the wall
            self.map.add((self.x + dx, self.y + dy), VS.OBST_WALL, VS.NO_VICTIM, self.check_walls_and_lim())
            #print(f"{self.NAME}: Wall or grid limit reached at ({self.x + dx}, {self.y + dy})")

        if result == VS.EXECUTED:

            self.cells_known[self.__get_current_pos()]["visited"] = True

            # check for victim returns -1 if there is no victim or the sequential
            # the sequential number of a found victim
            self.walk_stack.push((dx, dy))

            # update the agent's position relative to the origin
            self.x += dx
            self.y += dy          

            # Check for victims
            seq = self.check_for_victim()
            if seq != VS.NO_VICTIM:
                vs = self.read_vital_signals()
                self.victims[vs[0]] = ((self.x, self.y), vs)
                print(f"{self.NAME} Victim found at ({self.x}, {self.y}), rtime: {self.get_rtime()}")
                #print(f"{self.NAME} Seq: {seq} Vital signals: {vs}")
            
            # Calculates the difficulty of the visited cell
            difficulty = (rtime_bef - rtime_aft)
            if dx == 0 or dy == 0:
                difficulty = difficulty / self.COST_LINE
            else:
                difficulty = difficulty / self.COST_DIAG

            # Update the map with the new cell
            self.map.add((self.x, self.y), difficulty, seq, self.check_walls_and_lim())
            #print(f"{self.NAME}:at ({self.x}, {self.y}), diffic: {difficulty:.2f} vict: {seq} rtime: {self.get_rtime()}")

        return

    def come_back(self):
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
            #print(f"{self.NAME}: coming back at ({self.x}, {self.y}), rtime: {self.get_rtime()}")
        
    def deliberate(self) -> bool:
        """ The agent chooses the next action. The simulator calls this
        method at each cycle. Must be implemented in every agent"""

        consumed_time = self.TLIM - self.get_rtime()
        if consumed_time < self.get_rtime():
            self.explore()
            return True

        # time to come back to the base
        if self.walk_stack.is_empty() or (self.x == 0 and self.y == 0):
            # time to wake up the rescuer
            # pass the walls and the victims (here, they're empty)
            print(f"{self.NAME}: rtime {self.get_rtime()}, invoking the rescuer")
            #input(f"{self.NAME}: type [ENTER] to proceed")
            self.resc.go_save_victims(self.map, self.victims)
            return False

        self.come_back()
        return True

