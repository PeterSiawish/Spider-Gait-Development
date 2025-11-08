from math import pi
import random as rd
import time
import numpy as np
import scipy.io as sio  # For saving the best gait to a .mat file
from utility_functions import (
    initialize_gait,
    fitness,
    tournament_selection,
    crossover,
    mutate,
)


# Start by identifying what a possible chromosome could be.
# Since the MATLAB function accepts 1x24 vectors, a logical starter could be a list of 24 floating-point numbers, each representing a joint angle in radians.
# chromosome_list: list[float] = []

# Another candidate could be a dictionary with 24 key-value pairs, where keys are strings representing joint names and values are angles in radians.
# chromosome_dict: dict[str, float] = {}

# But this would generate 1 pose of the spider only. We need around 300 of these 'positions/frames' to generate a full Gait.
# In addition to generating ~300 poses, they will need to logically continue from one to the next. Mutations and other GA operations may result in abrupt changes between poses, leading to unnatural movements.
# Therefore, a more complex approach is needed.

# One obvious path we can consider is the naive approach of having a list of 300 x 24 = 7200 floating-point numbers, where each set of 24 numbers represents a pose at a specific time frame. The following codebase will explore this approach.

# A 300 x 24 list would look like this:
# chromosome_2d_list: list[list[float]] = []
# Each inner list would contain 24 floating-point numbers representing joint angles for a specific pose (indices from 0-299 would represent the 300 poses required for the full gait.) We can then define our chromosome as a 300x24 structure that contains a full gait. However, this 300x24 structure represents a single chromosome. In a genetic algorithm, we typically work with a population of such chromosomes. Therefore in order to create a suitable/reasonable population, we would need a large amount of these 300x24 chromosomes to form a population.


# Define a main function to encapsulate the various processes alongside benchmarking and testing (all other functions with explanations will be defined in utility_functions.py):
def main():
    population_size = 2000
    num_generations = 300
    population = []
    fitness_scores = []
    # The 'fitness_scores' list will hold fitness scores for each chromosome in the population, so it would have a length corresponding to the population size. Keep in mind a chromosome is the same as a full gait.

    # Initialize the population with random chromosomes (gaits)
    for _ in range(population_size):
        chromosome = initialize_gait()
        # Refer to the initialize_gait() function in utility_functions.py for more details.
        population.append(chromosome)

    print(f"Initialized population with {population_size} chromosomes...")

    fitness_scores = [fitness(member) for member in population]

    # Evolve the population over a number of generations
    for generation in range(num_generations):
        new_population = []

        while len(new_population) < population_size:
            parent1 = tournament_selection(population, fitness_scores)
            parent2 = tournament_selection(population, fitness_scores)
            child1, child2 = crossover(parent1, parent2)
            child1 = mutate(child1)
            child2 = mutate(child2)
            new_population.extend([child1, child2])

        population = new_population[:population_size]

        fitness_scores = [fitness(member) for member in population]

        print(f"Generation #{generation + 1} completed...")
        print(f"Best fitness in Generation #{generation + 1}: {max(fitness_scores)}")

    print("===================================")
    print(f"\nEvolution completed after {num_generations} generations.")
    best_member_index = fitness_scores.index(max(fitness_scores))
    best_gait = population[best_member_index]
    print(f"Best fitness achieved: {fitness_scores[best_member_index]}")
    print("\n===================================")
    print("Saving the best gait to 'best_gait.npy'...")
    np.save("best_gait.npy", best_gait)


# Run the main function if this script is executed directly (standard Python practice)
if __name__ == "__main__":
    # rd.seed(42)  # For reproducibility during testing and debugging
    start_time = time.perf_counter()
    main()
    end_time = time.perf_counter()
    execution_time = end_time - start_time
    print(f"\nTotal execution time: {execution_time:.4f} seconds")
