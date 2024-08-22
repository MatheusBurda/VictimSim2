import random
import string

TARGET = "algoritmo genetico"
POPULATION_SIZE = 100
MUTATION_RATE = 0.01
GENERATIONS = 1000

class GeneticAlgorithm:

    def __init__(self, cluster) -> None:
        self.cluster = cluster
        self.distances = []
        

    def create_individual(self):
        return random.shuffle(list(self.cluster.keys()))

    def fitness(self, individual):
        return sum(1 for a, b in zip(individual, TARGET) if a == b)

    def select_parent(self, population):
        weights = [fitness(individual) for individual in population]
        return random.choices(population, weights=weights, k=1)[0]

    def crossover(self, parent1, parent2):
        point = random.randint(0, len(parent1) - 1)
        child = parent1[:point] + parent2[point:]
        return child

    def mutate(self, individual):
        individual = list(individual)
        for i in range(len(individual)):
            if random.random() < MUTATION_RATE:
                individual[i] = random.choice(string.ascii_lowercase + ' ')
        return ''.join(individual)

    def run(self, ):
        population = [create_individual(len(TARGET)) for _ in range(POPULATION_SIZE)]
        
        for generation in range(GENERATIONS):
            population = sorted(population, key=lambda x: fitness(x), reverse=True)
            
            if fitness(population[0]) == len(TARGET):
                print(f"Encontrado na geração {generation}: {population[0]}")
                break
            
            new_population = population[:2]
            
            while len(new_population) < POPULATION_SIZE:
                parent1 = select_parent(population)
                parent2 = select_parent(population)
                child = crossover(parent1, parent2)
                child = mutate(child)
                new_population.append(child)
            
            population = new_population
            
            print(f"Geração {generation}, Melhor Indivíduo: {population[0]}, Aptidão: {fitness(population[0])}")

        else:
            print(f"Não foi encontrado um indivíduo perfeito em {GENERATIONS} gerações.")

if __name__ == "__main__":
    genetic_algorithm = GeneticAlgorithm()
    genetic_algorithm.run()
