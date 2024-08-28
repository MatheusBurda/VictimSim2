import random
import string

TARGET = 100
POPULATION_SIZE = 100
MUTATION_RATE = 0.4
CROSSOVER_RATE = 1
GENERATIONS = 1000
STOP_INALTERABILITY_COND = 50

class GeneticAlgorithm:

    def __init__(self, victims, victims_list) -> None:
        self.victims = victims
        self.victims_list = victims_list
        self.distances = []


    def create_individual(self):
        victims_copy = self.victims_list.copy()
        random.shuffle(victims_copy)
        return victims_copy


    def fitness(self, individual):
        """
        Calculates the fitness based on Manhattan distance and Gravity
        TODO : Implement a logic that takes in account the gravity of the victims
        """

        # Has repeated individuals
        if len(individual) != len(set(individual)):
            print('Individuo bugado: ')
            return 0
              
        total_distance = 0
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
        fitness = 1 / total_distance * 100 + total_grav

        return fitness


    def select_parent(self, population):
        weights = [self.fitness(individual) for individual in population]
        return random.choices(population, weights=weights, k=1)[0]


    def crossover(self, parent1, parent2):

        if random.random() < CROSSOVER_RATE:
            point = random.randint(0, len(parent1) - 1)
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
        
        # Crossover dont occurs, return a random parent
        if not not random.getrandbits(1):
            return parent1 
        else:
            return parent2


    def mutate(self, individual):
        individual = list(individual)
        for _ in range(len(individual)):
            if random.random() < MUTATION_RATE:
                gene_1_idx = random.randint(0, len(individual) - 1)
                gene_2_idx = random.randint(0, len(individual) - 1)
                individual[gene_1_idx], individual[gene_2_idx] = individual[gene_2_idx], individual[gene_1_idx]

        return individual


    def run(self):
        population = [self.create_individual() for _ in range(POPULATION_SIZE)]

        best_individual = population[0]
        times_stucked = 0
        
        for generation in range(GENERATIONS):

            # Sorts the population by fitness
            population = sorted(population, key=lambda x: self.fitness(x), reverse=True)

            new_best_individual = population[0]
            
            # Stop conditions
            if self.fitness(new_best_individual) == TARGET:
                print(f"Target genratin finded {generation}: {new_best_individual}")
                break
            # Add a stop condition if the fitness get stucked from a number of generations
            elif self.fitness(new_best_individual) == self.fitness(best_individual):
                times_stucked += 1
                if times_stucked == STOP_INALTERABILITY_COND:
                    print(f"Population stucked at generation {generation}")
                    break
            else:
                times_stucked = 0
            
            # Select the best ones from current pop to continue on next generation
            new_population = population[:2]
            
            while len(new_population) < POPULATION_SIZE:
                parent1 = self.select_parent(population)
                parent2 = self.select_parent(population)
                child = self.crossover(parent1, parent2)
                child = self.mutate(child)
                new_population.append(child)
            
            population = new_population
            best_individual = population[0]
            
            print(f"gen: {generation} -> fitness: {self.fitness(best_individual)}")   
            # print(f"Geração {generation}, Melhor Indivíduo: {best_individual}, Aptidão: {self.fitness(best_individual)}")   

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
