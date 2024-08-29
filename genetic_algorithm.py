import random

from tqdm import tqdm

import matplotlib.pyplot as plt

from vs.abstract_agent import PriorityQueue

DEBUG = True  # set as false to remove plots

GENERATIONS = 1000
POPULATION_SIZE = 30

MUTATION_RATE = 0.6

FITNESS_DISTANCE_WEIGHT = 5 # [1, inf[
FITNESS_GRAVITY_WEIGHT = 20 # [0, inf[
FITNESS_BAD_SOLUTION_MULTIPLIER = 2

STOP_INALTERABILITY_COUNT = 50
MIN_IMPROVEMENT_REQUIRED = 0.001
TARGET = 100000

class GeneticAlgorithm():

    def __init__(self, victims, victims_list, cells_known, total_bat, COST_DIAG, COST_LINE) -> None:

        self.COST_DIAG = COST_DIAG
        self.COST_LINE = COST_LINE

        self.victims = victims
        self.victims_list = victims_list
        self.distances = []
        self.sum_grav = sum([victims[vic]["grav"] for vic in victims_list])
        self.max_grav = max([victims[vic]["grav"] for vic in victims_list])
        self.cells_known = cells_known
        self.cells_known_list = cells_known.keys()
        self.total_bat = total_bat
        self.paths = {} # key (x0, y0, xf, yf) (from): {"path": path, "cost": cost}

        for victim_coord in victims_list:
            dict_key_from_to = (0, 0, victim_coord[0], victim_coord[1])
            dict_key_to_from = (victim_coord[0], victim_coord[1], 0, 0)

            path = self.cells_known[victim_coord]["path_to_base"]
            cost = self.cells_known[victim_coord]["cost_to_base"]

            self.paths[dict_key_from_to] = {"path": path, "cost": cost}
            self.paths[dict_key_to_from] = {"path": path[::-1], "cost": cost}

        
    def create_individual(self):
        victims_copy = self.victims_list.copy()
        random.shuffle(victims_copy)

        victims = [(0,0)] + victims_copy + [(0,0)]

        individual = {
            "victims": victims,
            "fitness": 0,
            "fitness_norm": 0,
            "path": []
        }

        return individual


    def fitness(self, individual):
        """
        Calculates the fitness based on Manhattan distance and Gravity
        """

        victims_list = individual["victims"]

        total_distance = 0
        total_grav = 0
        battery_left = self.total_bat
        individual["fitness"] = 0
        individual["path"] = [] 
        saved_all_victims = True

        # Has repeated individuals
        # if len(victims_list) != len(set(victims_list)) - 1:
        #     print('Doesnt contain all victims:')
        #     print(victims_list)
        #     print(set(victims_list))
        #     return

        # print(f'{len(victims_list)} : {victims_list}')
        for index, _ in enumerate(victims_list):
            if index == len(victims_list) - 1:
                break

            origin_coord = victims_list[index]
            goal_coord = victims_list[index + 1]

            dict_key_from_to = (origin_coord[0], origin_coord[1], goal_coord[0], goal_coord[1])

            # print(f'from -> to : {dict_key_from_to}')

            if dict_key_from_to not in self.paths.keys():
                path, cost = self.a_star_search(start=origin_coord, goal=goal_coord, cells_dict=self.cells_known)

                if path == [] and cost == -1:
                    # Path not found, impossible solution, unfitted solution
                    individual["fitness"] = 0
                    print('Path not found, impossible solution, unfitted solution')
                    return

                self.paths[dict_key_from_to] = {"path": path, "cost": cost}

                dict_key_to_from = (goal_coord[0], goal_coord[1], origin_coord[0], origin_coord[1])
                self.paths[dict_key_to_from] = {"path": path[::-1], "cost": cost}

            # Checks if the rescuer has battery to do the action and go back to the base with battery
            if goal_coord != (0,0) and (self.paths[dict_key_from_to]["cost"] + self.paths[(goal_coord[0], goal_coord[1], 0, 0)]["cost"] < battery_left):
            
                if len(individual["path"]) > 0 and individual["path"][-1] == self.paths[dict_key_from_to]["path"][0]:
                    individual["path"] += self.paths[dict_key_from_to]["path"][1:]
                else:
                    individual["path"] += self.paths[dict_key_from_to]["path"]
                
                battery_left -= self.paths[dict_key_from_to]["cost"]

                total_distance += self.paths[dict_key_from_to]["cost"]
                # The bigger the gravity of the individual first on the list
                total_grav += self.victims[victims_list[index + 1]]["grav"] / (index + 2)**2
            else: 
                individual["path"] += self.paths[(origin_coord[0], origin_coord[1], 0, 0)]["path"]
                total_distance += self.paths[(origin_coord[0], origin_coord[1], 0, 0)]["cost"]
                
                if goal_coord != (0,0): # Check if it was not going to the base
                    saved_all_victims = False

                break

        if total_distance == 0:
            print(individual)

        # Fitness is inversely proportional to distance
        fitness = (FITNESS_DISTANCE_WEIGHT * 1000 / total_distance) + (FITNESS_GRAVITY_WEIGHT * 100 * total_grav / self.sum_grav)

        if not saved_all_victims:
            remaining_victims = [self.victims[ idx ]["grav"] for idx in victims_list if (idx != (0,0) and idx not in individual["path"] and self.victims[ idx ]["grav"] > 60)]
            remaining_victims_grav = sum(remaining_victims)
            
            if len(remaining_victims) > 0:
                fitness -= FITNESS_BAD_SOLUTION_MULTIPLIER * remaining_victims_grav / len(remaining_victims)

        if fitness < 0:
            fitness = 0

        individual["fitness"] = fitness

        return


    def normalize_fitness(self, population):
        fitness_list = [ind["fitness"] for ind in population]

        sum_fit = sum(fitness_list)
        max_fit = max(fitness_list)
        
        if max_fit < 1:
            for idv in population:
                print(idv)

        for i in range(len(population)):
            population[i]["fitness_norm"] = population[i]["fitness"] / max_fit

        return population


    def select_parents(self, population):
        
        weights = [ind["fitness_norm"] for ind in population]

        selected = random.choices(population, weights=weights, k=2)

        return selected


    def crossover(self, indv1, indv2):

        parent1 = indv1["victims"]
        parent2 = indv2["victims"]

        point = random.randint(2, len(parent1) - 2)
        child = parent1[1:point] + parent2[point:-1]
        
        # checking for missing values and make them random
        child_set = set(child)
        missing = [victim for victim in self.victims_list if victim not in child_set] 
        random.shuffle(missing)

        # fill child with the missing values
        victims_set = set()
        for index, victim in enumerate(child):
            if victim in victims_set and victim != (0,0):
                child[index] = missing.pop()

            victims_set.add(child[index])

        child_mutated = self.mutate(child)

        # if child_mutated[1] == (0,0):
        #     print(f'\nchild: {child}')
        #     print(f'child_mutated: {child_mutated}')
        #     print(f'point: {point}')
        #     print(f'parent1: {parent1}')
        #     print(f'parent2: {parent2}\n')

        child_dict = {
            "victims": [(0,0)] + child_mutated + [(0,0)],
            "fitness": 0,
            "fitness_norm": 0,
            "path": []
        }

        return child_dict
        

    def mutate(self, individual):
        num_random_integers = 2 * int(len(individual) * MUTATION_RATE)

        random_indexes = [random.randint(0, len(individual) - 1) for _ in range(num_random_integers)]

        for i in range(0, len(random_indexes), 2):
            gene_1_idx = random_indexes[i]
            gene_2_idx = random_indexes[i + 1]
            individual[gene_1_idx], individual[gene_2_idx] = individual[gene_2_idx], individual[gene_1_idx]

        return individual
    

    def run(self):

        print(f'**************************************')
        print(f'Genetic algorithm params:\n')
        print(f'MAX number of generations: {GENERATIONS}')
        print(f'population size: {POPULATION_SIZE}')
        print(f'mutation rate: {MUTATION_RATE}')
        print(f'distance weight on fitness: {FITNESS_DISTANCE_WEIGHT}')
        print(f'gravity weight on fitness: {FITNESS_GRAVITY_WEIGHT}')
        print(f'gravity sum: {self.sum_grav}')
        print(f'**************************************')

        population = [self.create_individual() for _ in range(POPULATION_SIZE)]

        best_individual = population[0]
        times_stucked = 0
        
        min_values = []
        avg_values = []
        max_values = []
        best_values = []

        for generation in tqdm(range(GENERATIONS)):

            # Sorts the population by fitness
            population_fitness = []
            for indv in population:
                self.fitness(indv)
                population_fitness.append(indv["fitness"])
            
            population = self.normalize_fitness(population)

            curr_min = min(population_fitness)
            curr_avg = sum(population_fitness) / len(population_fitness)
            curr_max = max(population_fitness)
            index_max = population_fitness.index(curr_max)

            new_best_individual = population[index_max]
           
            # Stop conditions
            if best_individual["fitness"] >= TARGET:
                print(f"Target genratin finded {generation}: {best_individual}")
                break
            # Add a stop condition if the fitness get stucked from a number of generations
            # elif abs(best_individual["fitness"] - population_fitness[index_max]) < MIN_IMPROVEMENT_REQUIRED:
            #     times_stucked += 1
            #     if times_stucked >= STOP_INALTERABILITY_COUNT:
            #         print(f"Population stucked at generation {generation}")
            #         break
            else:
                times_stucked = 0

            if population_fitness[index_max] > best_individual["fitness"]:
                best_individual = new_best_individual
                best_individual["fitness"] = population_fitness[index_max]

            if DEBUG:
                min_values.append(curr_min)
                avg_values.append(curr_avg)
                max_values.append(curr_max)
                best_values.append(best_individual["fitness"])
            
            # Select the best one from current pop and the best overall to continue on next generation
            new_population = [population[index_max]]
            # new_population.append(self.mutate(best_individual))

            while len(new_population) < POPULATION_SIZE:
                parent1, parent2 = self.select_parents(population)
                child = self.crossover(parent1, parent2)
                # child = self.mutate(child)
                new_population.append(child)
            
            population = new_population
            
            # print(f"gen: {generation} -> fitness: {self.fitness(best_individual)}")   

        if DEBUG:
            plt.figure()

            time = range(len(max_values))
            plt.plot(time, max_values, color='blue', label='Max Value')
            plt.plot(time, min_values, color='red', label='Min Value')
            plt.plot(time, avg_values, color='green', label='Average')
            plt.plot(time, best_values, color='orange', label='Best')
            
            plt.xlabel('Generation')
            plt.ylabel('Fitness')
            plt.title('Fitness of Genetic Algorithm')
            plt.legend()

            plt.show()

        return best_individual  

    def update_costs(self, current_point, next_point, cells_dict=None):
        dx = current_point[0] - next_point[0]
        dy = current_point[1] - next_point[1]
        
        if not cells_dict:
            cells_dict = self.cells_known
        
        difficulty = cells_dict[next_point]["difficulty"]

        if dx == 0 or dy == 0:
            return difficulty * self.COST_LINE
        else:
            return difficulty * self.COST_DIAG

    def a_star_search(self, start, goal, cells_dict=None):

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
        
        if not cells_dict:
            cells_dict = self.cells_known

        assert type(cells_dict) == dict, 'cells_dict invalid or self.cells_known is not a dict'

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
                new_cost = cost_so_far[current] + self.update_costs(current, next, cells_dict)
                if next not in cost_so_far.keys() or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current

        if goal not in came_from:
            return [], -1

        path = reconstruct_path(came_from, start, goal)

        return path, cost_so_far[goal]