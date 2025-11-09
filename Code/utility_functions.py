from math import pi
import random as rd


# Below is a function to initialize a chromosome with random values within a specified range. The range boundaries will be explained below as well. Recall that each full gait is a single chromosome.
def initialize_gait():
    # The 'gait' list will be returned as a chromosome
    gait = []
    num_of_poses = 300
    for _ in range(num_of_poses):
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
def fitness(chromosome):
    # For demonstration purposes, let's define a simple fitness function that rewards gaits with smoother transitions between poses and symmetry.

    total_smoothness_penalty = 0
    total_symmetry_penalty = 0
    total_leg_crossing_penalty = 0

    num_poses = len(chromosome)  # Should be 300

    # First, let's define a smoothness metric: penalize large changes in joint angles between consecutive poses.
    for i in range(num_poses - 1):
        current_pose = chromosome[i]
        next_pose = chromosome[i + 1]
        pose_smoothness_penalty = 0.0

        for j in range(len(current_pose)):
            angle_difference = current_pose[j] - next_pose[j]
            pose_smoothness_penalty += angle_difference**2

        total_smoothness_penalty += pose_smoothness_penalty

    # Next, let's define a symmetry metric: penalize big differences between left and right legs.
    # Recall which joints mirror each other from the spider figure provided.
    # L1a (index 0) & R1a (index 21), L2a (index 3) & R2a (index 18), L3a (index 6) & R3a (index 15), L4a (index 9) & R4a (index 12 ).
    # Note that only Coxa joints will be used to measure the symmetry, as the relative positions of the tibia and femurs are allowed some flexibility.
    # So we will compare these pairs across all poses.
    mirror_joints_indices = [(0, 21), (3, 18), (6, 15), (9, 12)]
    for pose in chromosome:
        pose_symmetry_penalty = 0
        for left_index, right_index in mirror_joints_indices:
            left_coxa = pose[left_index]
            right_coxa = pose[right_index]
            symmetry_difference = left_coxa - right_coxa
            pose_symmetry_penalty += symmetry_difference**2
        total_symmetry_penalty += pose_symmetry_penalty

    # We can also add a penalty for leg crossing, which does not occur in natural spider gaits.
    for pose in chromosome:
        coxa_angles = [pose[i] for i in range(0, len(pose), 3)]
        left_coxa_angles = coxa_angles[:4]  # Indices for L1a, L2a, L3a, L4a
        right_coxa_angles = coxa_angles[4:]  # Indices for R1a, R2a, R3a, R4a
        pose_leg_crossing_penalty = 0

        for i in range(len(left_coxa_angles) - 1):
            diff = abs(left_coxa_angles[i] - left_coxa_angles[i + 1])
            if diff < 0.25:  # threshold: how close angles can get (≈14°)
                pose_leg_crossing_penalty += (0.25 - diff) ** 2

        for i in range(len(right_coxa_angles) - 1):
            diff = abs(right_coxa_angles[i] - right_coxa_angles[i + 1])
            if diff < 0.25:
                pose_leg_crossing_penalty += (0.25 - diff) ** 2

        total_crossing_penalty += pose_leg_crossing_penalty

    # We take the averages of the penalty so that the penalty is not affect by the number of chromosomes. Otherwise, the fitness of a population for 5000 would behave differently from a population of 1000.
    average_smoothness_penalty = total_smoothness_penalty / (num_poses - 1)
    average_symmetry_penalty = total_symmetry_penalty / num_poses
    average_leg_crossing_penalty = total_leg_crossing_penalty / num_poses

    # Add a weighting system to determine what is more desired from the gait:
    total_penalty = (
        75 * average_smoothness_penalty
        + 50 * average_symmetry_penalty
        + 75 * average_leg_crossing_penalty
    )

    # Finally, convert the penalty into a fitness score. A lower penalty should yield a higher fitness score, which is why we invert it here. The reason the numerator is 10000.0 is to scale the fitness score to a more manageable and human-readable number. It does not affect the relative fitness between different chromosomes because they are all scaled by the same factor.
    fitness = 10000.0 / (1.0 + total_penalty)

    return fitness


# Next, we will need a suitable selection function. For the purposes of this project, we can use tournament selection because it is simple to implement, understand, and provides good selection pressure.
def tournament_selection(population, fitness_scores, tournament_size=3):
    # First, we randomly select 'tournament_size/k' individuals from the population
    selected_indices = rd.sample(range(len(population)), tournament_size)

    # Once a random subset is selected, we determine the individual with the highest fitness score among them.
    # We pass a lambda function to the max() function to compare them based on their fitness scores.
    best_index = max(selected_indices, key=lambda i: fitness_scores[i])

    # With the best index/individual identified, we return the corresponding chromosome from the population as a deep copy to avoid unintended modifications to the original population.
    return [pose[:] for pose in population[best_index]]


# For the crossover, a one-point crossover can be implemented. This involves selecting a random crossover point along the chromosome and swapping the segments after that point between two parent chromosomes to create two offspring. This will be done at the pose level to maintain the integrity of each pose. A more advanced crossover method is possible, but such complexity is not necessary for a basic start.
def crossover(parent1, parent2):
    num_poses = len(parent1)
    crossover_point = rd.randint(1, num_poses - 2)

    child1 = parent1[:crossover_point] + parent2[crossover_point:]
    child2 = parent2[:crossover_point] + parent1[crossover_point:]

    # Make sure the children are returned as deep copies to prevent referencing bugs.
    child1 = [pose[:] for pose in child1]
    child2 = [pose[:] for pose in child2]

    return child1, child2


# Finally, we need a mutation function to introduce random variations into the chromosomes. This helps maintain genetic diversity within the population and allows the algorithm to explore a broader search space. A method we could use is simply adding or subtracting a small random value within a reasonable range (like -0.1 to 0.1 radians) to a randomly selected joint angle in a randomly selected pose.
def mutate(chromosome, mutation_rate=0.01, mutation_strength=0.1):
    # The mutation strength is the value that will added or subtracted to the individual angles if they are to be mutated.

    COXA_MIN, COXA_MAX = -pi / 2, pi / 2
    FEMUR_TIBIA_MIN, FEMUR_TIBIA_MAX = -2 * pi / 3, 0
    # We take the boundaries of the joints to ensure that the mutated angles remain within valid ranges.

    # Make a deep copy of the chromosome, which will be returned as the new chromosome.
    mutated_chromosome = [pose[:] for pose in chromosome]

    for pose_index in range(len(mutated_chromosome)):
        for joint_index in range(len(mutated_chromosome[pose_index])):
            if rd.random() < mutation_rate:
                mutated_chromosome[pose_index][joint_index] += rd.uniform(
                    -mutation_strength, mutation_strength
                )

                # If the joint index is a multiple of 3 (0, 3, 6, ...), it corresponds to a Coxa joint.
                if joint_index % 3 == 0:
                    mutated_chromosome[pose_index][joint_index] = max(
                        COXA_MIN,
                        min(COXA_MAX, mutated_chromosome[pose_index][joint_index]),
                    )
                else:
                    mutated_chromosome[pose_index][joint_index] = max(
                        FEMUR_TIBIA_MIN,
                        min(
                            FEMUR_TIBIA_MAX,
                            mutated_chromosome[pose_index][joint_index],
                        ),
                    )

    return mutated_chromosome
