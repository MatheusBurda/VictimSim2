    ##  RESCUER AGENT
### @Author: Tacla (UTFPR)
### Demo of use of VictimSim
### Not a complete version of DFS; it comes back prematuraly
### to the base when it enters into a dead end position


import os
import math
import random
from map import Map
from vs.abstract_agent import AbstAgent
from vs.physical_agent import PhysAgent
from vs.constants import VS
from abc import ABC, abstractmethod
from genetic_algorithm import GeneticAlgorithm
import heapq
from tqdm import tqdm

import regressor
import pandas as pd

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

## Classe que define o Agente Rescuer com um plano fixo
class Rescuer(AbstAgent):
    def __init__(self, env, config_file, captain=None):
        """ 
        @param env: a reference to an instance of the environment class
        @param config_file: the absolute path to the agent's config file"""

        super().__init__(env, config_file)

        # Specific initialization for the rescuer
        self.map = None             # explorer will pass the map
        # self.victims = None         # list of found victims
        self.plan = []              # a list of planned actions
        self.plan_x = 0             # the x position of the rescuer during the planning phase
        self.plan_y = 0             # the y position of the rescuer during the planning phase
        self.plan_visited = set()   # positions already planned to be visited 
        self.plan_rtime = self.TLIM # the remaing time during the planning phase
        self.plan_walk_time = 0.0   # previewed time to walk during rescue
        self.x = 0                  # the current x position of the rescuer when executing the plan
        self.y = 0                  # the current y position of the rescuer when executing the plan
        # Starts in IDLE state.
        # It changes to ACTIVE when the map arrives
        self.set_state(VS.IDLE)

        # Created atributes:
        self.captain = captain # Flag to indicate who is the captain to lead the other rescuers to their victims
        self.cells_known = {}
        self.victims = {}
        self.received_maps = 0
        self.rescuers = []
        self.n_resc = 1

        if self.captain: #register
            self.captain.cap_register_resc(self)


    def cap_register_resc(self, new_rescuer):
        self.n_resc += 1
        if new_rescuer is None:
            raise Exception('Rescuer is None')
        self.rescuers.append(new_rescuer)
        print(f'CAPTAIN: {self.n_resc} rescuers registered')


    def cap_receive_map(self, cells_known, victims):
        """ The explorer sends the map containing the walls and
        victims' location. If all maps were received from explorers
        then, the victims are clustered and sent to the rescuers to start the rescue"""

        for key in victims.keys():
            if key not in self.victims.keys():
                    self.victims[key] = victims[key]
        
        for key in cells_known.keys():
            # Save only visited cells from map
            if cells_known[key]["visited"] and (key not in self.cells_known.keys()):
                    self.cells_known[key] = cells_known[key]

        self.received_maps += 1

        print(f'\nRescuer captain: {self.received_maps} maps out of {self.n_resc} received')

        # If the captain has received all the maps, cluster them and distribute through the rescuers
        if self.received_maps >= self.n_resc:

            self.update_joined_maps_cost()

            # self.draw_map()

            self.predict_gravity()

            victim_clusters, centroids = self.k_means_clustering(self.victims, self.n_resc)    

            self.save_cluster_metrics(victim_clusters, centroids)

            for i, resc in enumerate(self.rescuers):
                resc.go_save_victims(self.cells_known, self.victims, victim_clusters[i + 1])
            
            self.go_save_victims(self.cells_known, self.victims, victim_clusters[0])     


    def draw_map(self):
        for key, item in  self.cells_known.items():
            print(f'{key}: {item}')
        x_values = [key[0] for key in self.cells_known.keys()]
        y_values = [key[1] for key in self.cells_known.keys()]
        min_x = min(x_values)
        max_x = max(x_values)
        min_y = min(y_values)
        max_y = max(y_values)

        for y in range(min_y, max_y+1):
            st = ""
            for x in range(min_x, max_x+1):
                if (x, y) in self.cells_known.keys():
                    st += f'{int(self.cells_known[(x, y)]["cost_to_base"]):3d} '
                else:
                    st += "___ "
            print(st)


    def update_joined_maps_cost(self):

        # def get_adjacents(position):
        #     adjacents = []
        #     for pos in self.cells_known.keys():
        #         if abs(pos[0] - position[0]) <= 1 and abs(pos[1] - position[1]) <= 1:
        #             adjacents.append(pos)

        #     return adjacents
        
        # def flood_fill(current_pos, current_cost):

        #     if self.cells_known[current_pos]['cost_to_base'] is None or current_cost < self.cells_known[current_pos]['cost_to_base']:
        #         self.cells_known[current_pos]['cost_to_base'] = current_cost
        #     else: 
        #         current_cost = self.cells_known[current_pos]['cost_to_base']
            
        #     adjacents = get_adjacents(current_pos)
            
        #     for next_pos in adjacents:
        #         step_cost = self.update_costs(current_pos, next_pos)
        #         next_cost = self.cells_known[next_pos]['cost_to_base']
        #         if (next_cost is None) or (next_cost > current_cost + step_cost):
        #             flood_fill(next_pos, current_cost + step_cost)

        print('\nJoining maps and recalculating...')

        for key in self.cells_known.keys():
            self.cells_known[key]["cost_to_base"] = None

        # cost to base is 0
        self.cells_known[(0,0)]["cost_to_base"] = 0

        # flood_fill((0,0), 0)

        for vic_pos in tqdm(self.victims.keys(), desc='Calculating the cost to base from all victims'):
            path, cost = self.a_star_search(vic_pos, (0,0))
            self.cells_known[vic_pos]["cost_to_base"] = cost
            self.cells_known[vic_pos]["path_to_base"] = path
        
        print('Finished\n')



    def predict_gravity(self, model_filename='gradient_boosting_model.pkl'):
        
        model = regressor.load_model(model_filename)
        
        # victims[] -> (x, y): {'id': id, 'signals': [index, pSist, pDiast, qPA, pulso, freqResp]}
        victims_signals = [victim["signals"][-3:] for _, victim in self.victims.items()]

        data_frame = pd.DataFrame(victims_signals)
        data_frame.columns = [['qPA', 'pulso', 'freqResp']]

        grav = regressor.predict(model, data_frame)

        for i, key in enumerate(self.victims.keys()):
            assert victims_signals[i] == self.victims[key]["signals"][-3:], 'Wrong victim'

            self.victims[key]["grav"] = grav[i]       


    def k_means_clustering(self, victims, k, max_iterations=100):      

        locations = list(victims.keys())
        clusters = None

        centroids = random.sample(locations, k)

        for i in range(max_iterations):

            # Cluster Atribution
            clusters = [[] for _ in centroids]
            for point in locations:
                min_distance = float('inf')
                closest_centroid = None
                for i, centroid in enumerate(centroids):
                    dist = math.sqrt((point[0]-centroid[0])**2 + (point[1]-centroid[1])**2)
                    if dist < min_distance:
                        min_distance = dist
                        closest_centroid = i
                clusters[closest_centroid].append(point)

            # Centroid calculation
            new_centroids = []
            for cluster in clusters:
                if cluster:
                    x_sum = sum(point[0] for point in cluster)
                    y_sum = sum(point[1] for point in cluster)
                    centroid = (x_sum / len(cluster), y_sum / len(cluster))
                    new_centroids.append(centroid)

            # No change
            if centroids == new_centroids:
                break

            centroids = new_centroids
        
        return clusters, centroids
    

    def save_cluster_metrics(self, clusters, centroids):
        data_folder = self._AbstAgent__env.data_folder

        if not os.path.exists(os.path.join(data_folder, 'output')):
            os.mkdir(os.path.join(data_folder, 'output'))

        sse = 0
        silhuet = 0
        for i in range(len(centroids)):
            for j in range(i, len(centroids)):
                silhuet += (centroids[i][0]-centroids[j][0])**2 + (centroids[i][1]-centroids[j][1])**2

        for i, cluster in enumerate(clusters):
            with open(os.path.join(data_folder, 'output', f'cluster{i+1}.txt'), 'w+') as file:
                for point in cluster: 
                    sse += (point[0]-centroids[i][0])**2 + (point[1]-centroids[i][1])**2

                    victim = self.victims[point]
                    grav = victim['grav']
                    label = 1
                    #  𝑖𝑑, 𝑥, 𝑦, 0.0, 1 (id é a identificação da vítima, x e y, a posição dela e os dois últimos valores correspondem ao valor da gravidade e ao seu label)
                    file.write(f'{victim["id"]}, {point[0]}, {point[1]}, {grav}, {label}\n')

        with open(os.path.join(data_folder, 'output', f'metrics.txt'), 'w+') as file:
            file.write(f'SSE: {sse}\n')
            file.write(f'Silhuet: {silhuet}\n')
            

    def go_save_victims(self, cells_known, victims, victims_list):
        """ The captain sends the map containing the walls and
        victims' location. The rescuer becomes ACTIVE. From now,
        the deliberate method is called by the environment"""

        self.cells_known = cells_known
        self.victims = victims
        self.victims_list = victims_list

        print(f"\n\n*** R E S C U E R ***")
        # self.map = cells_known.keys()
        print(f"{self.NAME} Map received from the captain")
        # self.map.draw()

        print(f"{self.NAME} PLAN")
        self.__planner(victims_list)
        print(f"{self.NAME} END OF PLAN")
                    
        self.set_state(VS.ACTIVE)

    def update_costs(self, current_point, next_point):
        dx = current_point[0] - next_point[0]
        dy = current_point[1] - next_point[1]
        
        difficulty = self.cells_known[next_point]["difficulty"]
        difficulty = 1 if difficulty == None else difficulty

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

    
    def __planner(self, victims_list):
        """ A private method that calculates the walk actions in a OFF-LINE MANNER to rescue the
        victims. Further actions may be necessary and should be added in the
        deliberata method"""

        """ This plan starts at origin (0,0) and chooses the first of the possible actions in a clockwise manner starting at 12h.
        Then, if the next position was visited by the explorer, the rescuer goes to there. Otherwise, it picks the following possible action.
        For each planned action, the agent calculates the time will be consumed. When time to come back to the base arrives,
        it reverses the plan."""

        # This is a off-line trajectory plan, each element of the list is a pair dx, dy that do the agent walk in the x-axis and/or y-axis.
        # Besides, it has a flag indicating that a first-aid kit must be delivered when the move is completed.
        # For instance (0,1,True) means the agent walk to (x+0,y+1) and after walking, it leaves the kit.

        ga_inst = GeneticAlgorithm(self.victims, victims_list, self.cells_known, self.get_rtime(), self.COST_DIAG, self.COST_LINE)
        best_sequence_victims = ga_inst.run()

        # TODO - Calculates the path based on A* algorithm
        start = (0, 0)
        path = []
        total_cost = 0
        for victim in best_sequence_victims:
            goal = victim
            new_path, cost = self.a_star_search(start, goal)
            if new_path == []:
                print(f'Path not found from {start} to {goal}')
                continue
            path = new_path[:-1]+path
            total_cost += cost
            start = goal

        goal = (0,0)
        new_path, cost = self.a_star_search(start, goal)
        if new_path == []:
            print(f'Path not found from {start} to {goal}')
        path = new_path[:-1]+path
        total_cost += cost

        self.plan = path

        # Para cada coordenada do path, verificar se tem vitima e adicionar a ação de resgate
        for i, point in enumerate(self.plan):
            if point in victims_list:
                self.plan[i] = (point[0], point[1], True)
            else:
                self.plan[i] = (point[0], point[1], False)

        # Transformando o caminho em dx e dy para o agente andar
        self.plan = [(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1], path[i+1][2]) for i in range(len(path)-1)]

        # Push actions into the plan to come back to the base
        if self.plan == []:
            return
        
        
    def deliberate(self) -> bool:
        """ This is the choice of the next action. The simulator calls this
        method at each reasonning cycle if the agent is ACTIVE.
        Must be implemented in every agent
        @return True: there's one or more actions to do
        @return False: there's no more action to do """
        
        # No more actions to do
        if self.plan == []:  # empty list, no more actions to do
           #input(f"{self.NAME} has finished the plan [ENTER]")
           return False
    
        # Takes the first action of the plan (walk action) and removes it from the plan
        dx, dy, there_is_vict = self.plan.pop(0)
        #print(f"{self.NAME} pop dx: {dx} dy: {dy} vict: {there_is_vict}")

        # Walk - just one step per deliberation
        walked = self.walk(dx, dy)

        # Rescue the victim at the current position
        if walked == VS.EXECUTED:
            self.x += dx
            self.y += dy
            #print(f"{self.NAME} Walk ok - Rescuer at position ({self.x}, {self.y})")
            # check if there is a victim at the current position
            if there_is_vict:
                rescued = self.first_aid() # True when rescued
                if rescued:
                    print(f"{self.NAME} Victim rescued at ({self.x}, {self.y})")
                else:
                    print(f"{self.NAME} Plan fail - victim not found at ({self.x}, {self.y})")
        else:
            print(f"{self.NAME} Plan fail - walk error - agent at ({self.x}, {self.y})")
            
        #input(f"{self.NAME} remaining time: {self.get_rtime()} Tecle enter")

        return True


