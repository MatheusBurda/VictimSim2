import random
import string

# Parâmetros do Algoritmo Genético
TARGET = "algoritmo genetico"
POPULATION_SIZE = 100
MUTATION_RATE = 0.01
GENERATIONS = 1000

# Função para criar um indivíduo aleatório
def create_individual(length):
    return ''.join(random.choices(string.ascii_lowercase + ' ', k=length))

# Função de aptidão (fitness) - compara o indivíduo com o alvo
def fitness(individual):
    return sum(1 for a, b in zip(individual, TARGET) if a == b)

# Função para selecionar um pai da população baseado na aptidão
def select_parent(population):
    weights = [fitness(individual) for individual in population]
    return random.choices(population, weights=weights, k=1)[0]

# Função de crossover entre dois pais
def crossover(parent1, parent2):
    point = random.randint(0, len(parent1) - 1)
    child = parent1[:point] + parent2[point:]
    return child

# Função de mutação de um indivíduo
def mutate(individual):
    individual = list(individual)
    for i in range(len(individual)):
        if random.random() < MUTATION_RATE:
            individual[i] = random.choice(string.ascii_lowercase + ' ')
    return ''.join(individual)

# Função principal do algoritmo genético
def genetic_algorithm():
    population = [create_individual(len(TARGET)) for _ in range(POPULATION_SIZE)]
    
    for generation in range(GENERATIONS):
        population = sorted(population, key=lambda x: fitness(x), reverse=True)
        
        if fitness(population[0]) == len(TARGET):
            print(f"Encontrado na geração {generation}: {population[0]}")
            break
        
        new_population = population[:2]  # Preservar os melhores indivíduos
        
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
    genetic_algorithm()
