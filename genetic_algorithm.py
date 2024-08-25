import random
import string

TARGET = 100
POPULATION_SIZE = 100
MUTATION_RATE = 0.01
GENERATIONS = 10

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
        total_distance = 0
        for index, _ in enumerate(individual):
            if index == len(individual) - 1:
                break
            x1, y1 = individual[index]
            x2, y2 = individual[index + 1]
            distance = abs(x1 - x2) + abs(y1 - y2)
            if distance == 0:
                contains_zero = True
                break
            total_distance += distance
        
        # If one of the distance is 0, the crossover generated a bad individual, so the fitness receives 0
        if "contains_zero" in locals() and contains_zero == True :
            fitness = 0
        else:
            # Fitness is inversely proportional to distance
            if total_distance == 0:
                fitness = 100
            else:
                fitness = 1 / total_distance * 100
        
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
        
        for generation in range(GENERATIONS):

            # Sorts the population by fitness
            population = sorted(population, key=lambda x: self.fitness(x), reverse=True)
            
            if self.fitness(population[0]) == TARGET:
                print(f"Target genratin finded {generation}: {population[0]}")
                break
            
            new_population = population[:2]
            
            # Creates new population based on the parents, crossover and mutation
            while len(new_population) < POPULATION_SIZE:
                parent1 = self.select_parent(population)
                parent2 = self.select_parent(population)
                child = self.crossover(parent1, parent2)
                child = self.mutate(child)
                new_population.append(child)
            
            population = new_population
            
            # print(f"Geração {generation}, Melhor Indivíduo: {population[0]}, Aptidão: {self.fitness(population[0])}")   

        return population[0]   

if __name__ == "__main__":

    # list of tuples with the coordinates of the victims
    cluster = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10), (11, 12), (13, 14), (15, 16), (17, 18), (19, 20)]
    victims = [(1, 2), (7, 8), (5, 6), (3, 4)]

    genetic_algorithm = GeneticAlgorithm(cluster=cluster, victims=victims)
    genetic_algorithm.run()
