import random

from tqdm import tqdm

import matplotlib.pyplot as plt

DEBUG = False  # set as false to remove plots

GENERATIONS = 1000
POPULATION_SIZE = 100

MUTATION_RATE = 0.1
CROSSOVER_RATE = 0.8

FITNESS_DISTANCE_WEIGHT = 1 # [1, inf[
FITNESS_GRAVITY_WEIGHT = 1 # [0, inf[

STOP_INALTERABILITY_COUNT = 50
MIN_IMPROVEMENT_REQUIRED = 0.01
TARGET = 100000

class GeneticAlgorithm:

    def __init__(self, victims, victims_list) -> None:
        self.victims = victims
        self.victims_list = victims_list
        self.distances = []
        self.sum_grav = sum([victims[vic]["grav"] for vic in victims_list])
        self.max_grav = max([victims[vic]["grav"] for vic in victims_list])


    def create_individual(self):
        victims_copy = self.victims_list.copy()
        random.shuffle(victims_copy)
        return victims_copy


    def fitness(self, individual):
        """
        Calculates the fitness based on Manhattan distance and Gravity
        """

        # Has repeated individuals
        if len(individual) != len(set(individual)):
            print('Individuo bugado: ')
            return 0
              
        total_distance = 1
        total_grav = self.victims[individual[0]]["grav"]

        for index, _ in enumerate(individual):
            if index == len(individual) - 1:
                break

            x1, y1 = individual[index]
            x2, y2 = individual[index + 1]
            total_distance += abs(x1 - x2) + abs(y1 - y2)

            # The bigger the gravity of the individual first on the list
            total_grav += self.victims[individual[index + 1]]["grav"] / (index + 2)

        # Fitness is inversely proportional to distance
        fitness = (FITNESS_DISTANCE_WEIGHT * 1000 / total_distance) + (FITNESS_GRAVITY_WEIGHT * 100 * total_grav / self.sum_grav)

        # print(f'fitness = {FITNESS_DISTANCE_WEIGHT * 1000 / total_distance} + {FITNESS_GRAVITY_WEIGHT * 100 * total_grav / self.sum_grav} = {fitness}')

        return fitness


    def normalize_fitness(self, fitness_list):
        sum_fit = sum(fitness_list)

        for i in range(len(fitness_list)):
            fitness_list[i] /= sum_fit

        return fitness_list


    def select_parents(self, weights, population):
        
        selected = random.choices(population, weights=weights, k=2)

        return selected


    def crossover(self, parent1, parent2):

        if random.random() < CROSSOVER_RATE:
            point = random.randint(1, len(parent1) - 1)
            child = parent1[:point] + parent2[point:]
            
            # checking for missing values and make them random
            child_set = set(child)
            missing = [victim for victim in self.victims_list if victim not in child_set] 
            random.shuffle(missing)

            # fill child with the missing values
            victims_set = set()
            for index, victim in enumerate(child):
                if victim in victims_set:
                    child[index] = missing.pop()

                victims_set.add(child[index])

            return child
        
        # Crossover dont occurs, choose a random parent
        if not not random.getrandbits(1):
            return parent1 
        else:
            return parent2


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
        print(f'crossover rate: {CROSSOVER_RATE}')
        print(f'distance weight on fitness: {FITNESS_DISTANCE_WEIGHT}')
        print(f'gravity weight on fitness: {FITNESS_GRAVITY_WEIGHT}')
        print(f'gravity sum: {self.sum_grav}')
        print(f'**************************************')

        population = [self.create_individual() for _ in range(POPULATION_SIZE)]

        best_individual = population[0]
        best_individual_fitness = 0
        times_stucked = 0
        
        min_values = []
        avg_values = []
        max_values = []
        best_values = []

        for generation in tqdm(range(GENERATIONS)):

            # Sorts the population by fitness
            population_fitness = [self.fitness(ind) for ind in population]

            population_fitness = self.normalize_fitness(population_fitness)

            curr_min = min(population_fitness)
            curr_avg = sum(population_fitness) / len(population_fitness)
            curr_max = max(population_fitness)
            index_max = population_fitness.index(curr_max)

            new_best_individual = population[index_max]
           
            # Stop conditions
            if best_individual_fitness >= TARGET:
                print(f"Target genratin finded {generation}: {best_individual}")
                break
            # Add a stop condition if the fitness get stucked from a number of generations
            elif abs(best_individual_fitness - population_fitness[index_max]) < MIN_IMPROVEMENT_REQUIRED:
                times_stucked += 1
                if times_stucked >= STOP_INALTERABILITY_COUNT:
                    print(f"Population stucked at generation {generation}")
                    break
            else:
                times_stucked = 0

            if population_fitness[index_max] > best_individual_fitness:
                best_individual = new_best_individual
                best_individual_fitness = population_fitness[index_max]

            if DEBUG:
                min_values.append(curr_min)
                avg_values.append(curr_avg)
                max_values.append(curr_max)
                best_values.append(best_individual_fitness)
            
            # Select the best one from current pop and the best overall to continue on next generation
            new_population = [self.mutate(population[index_max])]
            new_population.append(self.mutate(best_individual))

            while len(new_population) < POPULATION_SIZE:
                parent1, parent2 = self.select_parents(weights=population_fitness, population=population)
                child = self.crossover(parent1, parent2)
                child = self.mutate(child)
                new_population.append(child)
            
            population = new_population
            
            # print(f"gen: {generation} -> fitness: {self.fitness(best_individual)}")   
            # print(f"Geração {generation}, Melhor Indivíduo: {best_individual}, Aptidão: {self.fitness(best_individual)}")   

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


if __name__ == "__main__":

    grid_size = 50
    num_victims = 30

    # # Generates a sample grid
    # cluster = [(x, y) for x in range(grid_size) for y in range(grid_size)]

    # Randomly generate 100 victims
    victims = set()
    while len(victims) < num_victims:
        point = random.randint(0, grid_size - 1), random.randint(0, grid_size - 1)
        victims.add(point)

    # transform set into list
    victims = list(victims)
    
    genetic_algorithm = GeneticAlgorithm(victims=victims)
    best = genetic_algorithm.run()

    # compute the euclidean distance between the victims
    total_distance = 0
    for index, _ in enumerate(best):
        if index == len(best) - 1:
            break
        x1, y1 = best[index]
        x2, y2 = best[index + 1]
        total_distance += abs(x1 - x2) + abs(y1 - y2)
    
    print(f"Best Individual: {best}, Total Distance to Walk: {total_distance}")
