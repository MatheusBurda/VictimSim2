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
            if key not in self.cells_known.keys():
                    self.cells_known[key] = cells_known[key]

        self.received_maps += 1

        print(f'Captain: {self.received_maps} maps out of {self.n_resc} received')

        # If the captain has received all the maps, cluster them and distribute through the rescuers
        if self.received_maps >= self.n_resc:

            victim_clusters = self.k_means_clustering(self.victims, self.n_resc)    

            self.save_cluster_metrics(victim_clusters)

            for i, resc in enumerate(self.rescuers):
                resc.go_save_victims(self.cells_known, victim_clusters[i + 1])
            
            self.go_save_victims(self.cells_known, victim_clusters[0])     

    
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
        
        return clusters
    

    def save_cluster_metrics(self, clusters):
        data_folder = self._AbstAgent__env.data_folder

        if not os.path.exists(os.path.join(data_folder, 'output')):
            os.mkdir(os.path.join(data_folder, 'output'))

        for i, cluster in enumerate(clusters):
            with open(os.path.join(data_folder, 'output', f'cluster{i+1}.txt'), 'w+') as file:
                for point in cluster: 
                    victim = self.victims[point]
                    #TODO: Change:
                    grav = 0
                    label = 1
                    #  𝑖𝑑, 𝑥, 𝑦, 0.0, 1 (id é a identificação da vítima, x e y, a posição dela e os dois últimos valores correspondem ao valor da gravidade e ao seu label)
                    file.write(f'{victim["id"]}, {point[0]}, {point[1]}, {grav}, {label}\n')


    def go_save_victims(self, cells_known, victims):
        """ The captain sends the map containing the walls and
        victims' location. The rescuer becomes ACTIVE. From now,
        the deliberate method is called by the environment"""

        self.cells_known = cells_known

        print(f"\n\n*** R E S C U E R ***")
        # self.map = cells_known.keys()
        print(f"{self.NAME} Map received from the captain")
        # self.map.draw()

        self.set_state(VS.ACTIVE)

        return

        print()
        #print(f"{self.NAME} List of found victims received from the explorer")
        self.victims = victims

        self.__planner()
        print(f"{self.NAME} PLAN")
        i = 1
        self.plan_x = 0
        self.plan_y = 0
        for a in self.plan:
            self.plan_x += a[0]
            self.plan_y += a[1]
            print(f"{self.NAME} {i}) dxy=({a[0]}, {a[1]}) vic: a[2] => at({self.plan_x}, {self.plan_y})")
            i += 1

        print(f"{self.NAME} END OF PLAN")
                    
        self.set_state(VS.ACTIVE)


    def __depth_search(self, actions_res):
        enough_time = True
        ##print(f"\n{self.NAME} actions results: {actions_res}")
        for i, ar in enumerate(actions_res):

            if ar != VS.CLEAR:
                ##print(f"{self.NAME} {i} not clear")
                continue

            # planning the walk
            dx, dy = Rescuer.AC_INCR[i]  # get the increments for the possible action
            target_xy = (self.plan_x + dx, self.plan_y + dy)

            # checks if the explorer has not visited the target position
            if not self.map.in_map(target_xy):
                ##print(f"{self.NAME} target position not explored: {target_xy}")
                continue

            # checks if the target position is already planned to be visited 
            if (target_xy in self.plan_visited):
                ##print(f"{self.NAME} target position already visited: {target_xy}")
                continue

            # Now, the rescuer can plan to walk to the target position
            self.plan_x += dx
            self.plan_y += dy
            difficulty, vic_seq, next_actions_res = self.map.get((self.plan_x, self.plan_y))
            #print(f"{self.NAME}: planning to go to ({self.plan_x}, {self.plan_y})")

            if dx == 0 or dy == 0:
                step_cost = self.COST_LINE * difficulty
            else:
                step_cost = self.COST_DIAG * difficulty

            #print(f"{self.NAME}: difficulty {difficulty}, step cost {step_cost}")
            #print(f"{self.NAME}: accumulated walk time {self.plan_walk_time}, rtime {self.plan_rtime}")

            # check if there is enough remaining time to walk back to the base
            if self.plan_walk_time + step_cost > self.plan_rtime:
                enough_time = False
                #print(f"{self.NAME}: no enough time to go to ({self.plan_x}, {self.plan_y})")
            
            if enough_time:
                # the rescuer has time to go to the next position: update walk time and remaining time
                self.plan_walk_time += step_cost
                self.plan_rtime -= step_cost
                self.plan_visited.add((self.plan_x, self.plan_y))

                if vic_seq == VS.NO_VICTIM:
                    self.plan.append((dx, dy, False)) # walk only
                    #print(f"{self.NAME}: added to the plan, walk to ({self.plan_x}, {self.plan_y}, False)")

                if vic_seq != VS.NO_VICTIM:
                    # checks if there is enough remaining time to rescue the victim and come back to the base
                    if self.plan_rtime - self.COST_FIRST_AID < self.plan_walk_time:
                        print(f"{self.NAME}: no enough time to rescue the victim")
                        enough_time = False
                    else:
                        self.plan.append((dx, dy, True))
                        #print(f"{self.NAME}:added to the plan, walk to and rescue victim({self.plan_x}, {self.plan_y}, True)")
                        self.plan_rtime -= self.COST_FIRST_AID

            # let's see what the agent can do in the next position
            if enough_time:
                self.__depth_search(self.map.get((self.plan_x, self.plan_y))[2]) # actions results
            else:
                return

        return
    
    def __planner(self):
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

        self.plan_visited.add((0,0)) # always start from the base, so it is already visited
        difficulty, vic_seq, actions_res = self.map.get((0,0))
        self.__depth_search(actions_res)

        # push actions into the plan to come back to the base
        if self.plan == []:
            return

        come_back_plan = []

        for a in reversed(self.plan):
            # triple: dx, dy, no victim - when coming back do not rescue any victim
            come_back_plan.append((a[0]*-1, a[1]*-1, False))

        self.plan = self.plan + come_back_plan
        
        
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
                    print(f"{self.NAME} Plan fail - victim not found at ({self.x}, {self.x})")
        else:
            print(f"{self.NAME} Plan fail - walk error - agent at ({self.x}, {self.x})")
            
        #input(f"{self.NAME} remaining time: {self.get_rtime()} Tecle enter")

        return True

