import random
import string

TARGET = 100
POPULATION_SIZE = 100
MUTATION_RATE = 0.2
GENERATIONS = 1000
STOP_INALTERABILITY_COND = 250

class GeneticAlgorithm:

    def __init__(self, cluster, victims) -> None:
        self.cluster = cluster
        self.victims = victims
        self.distances = []

    def create_individual(self):
        victims_copy = self.victims.copy()
        random.shuffle(victims_copy)
        return victims_copy

    def fitness(self, individual):
        """
        Calculates the fitness based on Manhattan distance and Gravity
        TODO : Implement a logic that takes in account the gravity of the victims
        """
        
        # Verify if the individual has all the victims
        contain_all_vic = set(individual) == set(self.victims)
        if contain_all_vic == True:
        
            total_distance = 0
            for index, _ in enumerate(individual):
                if index == len(individual) - 1:
                    break
                x1, y1 = individual[index]
                x2, y2 = individual[index + 1]
                total_distance += abs(x1 - x2) + abs(y1 - y2)
                # Fitness is inversely proportional to distance
                fitness = 1 / total_distance * 100
        else:
            # If the individual does not contain all the victims, attribute a bad fitness
            fitness = 0

        # print(f"Individual: {individual}, Fitness: {fitness}")

        return fitness

    def select_parent(self, population):
        weights = [self.fitness(individual) for individual in population]
        return random.choices(population, weights=weights, k=1)[0]

    def crossover(self, parent1, parent2):
        point = random.randint(0, len(parent1) - 1)
        child = parent1[:point] + parent2[point:]
        return child

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
            
            new_population = population[:2]
            
            # Creates new population based on the parents, crossover and mutation
            while len(new_population) < POPULATION_SIZE:
                parent1 = self.select_parent(population)
                parent2 = self.select_parent(population)
                child = self.crossover(parent1, parent2)
                child = self.mutate(child)
                new_population.append(child)
            
            population = new_population
            best_individual = population[0]
            
            print(f"Geração {generation}, Melhor Indivíduo: {best_individual}, Aptidão: {self.fitness(best_individual)}")   

        return best_individual  

if __name__ == "__main__":

    grid_size = 50
    num_victims = 30

    # Generates a sample grid
    cluster = [(x, y) for x in range(grid_size) for y in range(grid_size)]

    # Randomly generate 100 victims
    victims = set()
    while len(victims) < num_victims:
        point = random.randint(0, grid_size - 1), random.randint(0, grid_size - 1)
        victims.add(point)

    # transform set into list
    victims = list(victims)
    
    genetic_algorithm = GeneticAlgorithm(cluster=cluster, victims=victims)
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
