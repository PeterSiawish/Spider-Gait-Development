from math import pi
import random as rd


# Below is a function to initialize a chromosome with random values within a specified range. The range boundaries will be explained below as well. Recall that each full gait is a single chromosome.
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
