from math import pi
import random as rd


# Below is a function to initialize a chromosome with random values within a specified range. The range boundaries will be explained below as well. Recall that each full gait is a single chromosome.
def initialize_gait():
    # The 'gait' list will be returned as a chromosome
    gait = []
    for _ in range(300):  # For 300 individual poses
        pose = []  # Create a new list for each pose
        for __ in range(8):  # 8 legs
            # Upon testing different possible angles for different joints, the following ranges make the most sense to use:
            # Coxa = -pi/2 to pi/2. Allows the leg to move left and right with respect to the body without the leg crossing into the spider's body.
            # Femur = -2pi/3 to 0. Allows the leg to curl naturally without 'breaking' the joint by bending backwards.
            # Tibia = -2pi/3 to 0. Same reasoning as Femur, but for the Tibia segment.
            coxa_angle = rd.uniform(-pi / 2, pi / 2)
            femur_angle = rd.uniform(-2 * pi / 3, 0)
            tibia_angle = rd.uniform(-2 * pi / 3, 0)
            pose.extend([coxa_angle, femur_angle, tibia_angle])
        gait.append(pose)

    return gait


# A key component of any genetic algorithm is the fitness function, which evaluates how 'fit' or 'good' a particular chromosome is. Below is a fitness function that evaluates a gait based on specific criteria.
def fitness(chromosome): ...
