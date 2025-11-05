from math import pi
import random as rd
import time

# Start by identifying what a possible chromosome could be.
# Since the MATLAB function accepts 1x24 vectors, a logical starter could be a list of 24 floating-point numbers, each representing a joint angle in radians.
chromosome_list: list[float] = []

# Another candidate could be a dictionary with 24 key-value pairs, where keys are strings representing joint names and values are angles in radians.
chromosome_dict: dict[str, float] = {}

# But this would generate 1 pose of the spider only. We need around 300 of these 'positions/frames' to generate a full Gait.
# In addition to generating ~300 poses, they will need to logically continue from one to the next. Mutations and other GA operations may result in abrupt changes between poses, leading to unnatural movements.
# Therefore, a more complex approach is needed.

# One obvious path we can consider is the naive approach of having a list of 300 x 24 = 7200 floating-point numbers, where each set of 24 numbers represents a pose at a specific time frame. The following codebase will explore this approach.

# A 300 x 24 list would look like this:
chromosome_2d_list: list[list[float]] = []
# Each inner list would contain 24 floating-point numbers representing joint angles for a specific pose (indices from 0-299 would represent the 300 poses required for the full gait.) We can then define our chromosome as a 300x24 structure that contains a full gait. However, this 300x24 structure represents a single chromosome. In a genetic algorithm, we typically work with a population of such chromosomes. Therefore in order to create a suitable/reasonable population, we would need a large amount of these 300x24 chromosomes to form a population.


# Below is a function to initialize such a chromosome with random values within a specified range. The range boundaries will be explained below as well. Recall that each full gait is a single chromosome.
def initialize_gait():
    # The 'gait' list will be returned as a chromosome
    gait = []
    for _ in range(300):  # For 300 individual poses
        pose = []  # Create a new list for each pose
        for __ in range(24):  # 24 joints
            # Since we are dealing with radians, the range will be from -π to π is a logical starting point. The fitness function can later determine ideal values for specific joints.
            angle = rd.uniform(-pi, pi)
            pose.append(angle)
        gait.append(pose)

    return gait


# A key component of any genetic algorithm is the fitness function, which evaluates how 'fit' or 'good' a particular chromosome is. Below is a fitness function that evaluates a gait based on specific criteria.
def fitness(chromosome): ...


# Define a main function to encapsulate the various processes alongside benchmarking and testing:
def main():
    population_size = 1000
    population = []

    for _ in range(population_size):
        chromosome = initialize_gait()
        population.append(chromosome)

    fitness_scores = []
    # The 'fitness_scores' list will hold fitness scores for each chromosome in the population, so it would have a length corresponding to the population size. Keep in mind a chromosome is the same as a full gait.
    for member in population:
        score = fitness(member)
        fitness_scores.append(score)


# Run the main function if this script is executed directly (standard Python practice)
if __name__ == "__main__":
    start_time = time.perf_counter()
    main()
    end_time = time.perf_counter()
    execution_time = end_time - start_time
    print(f"\nTotal execution time: {execution_time:.4f} seconds")
