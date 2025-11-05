from math import pi

# Start by identifying what a possible chromosome could be.
# Since the MATLAB function accepts 1x24 vectors, a logical starter could be a list of 24 floating-point numbers.
chromosome_list: list[float] = []

# Another candidate could be a dictionary with 24 key-value pairs, where keys are strings representing joint names and values are angles in radians.
chromosome_dict: dict[str, float] = {}

# But this would generate 1 pose of the spider only. We need around 300 of these 'positions/frames' to generate a full Gait.
# In addition to generating ~300 poses, they will need to logically continue from one to the next. Mutations and other GA operations may result in abrupt changes between poses, leading to unnatural movements.
# Therefore, a more complex approach is needed.

# One path we can consider is the use of a sinusoidal function to generate smooth transitions between poses.
# Using this parametric technique, each joint would be represented by 3 values according to the formula:
# angle(t) = A * sin(2 * pi * B * t + C) + D
# Where:
# A = Amplitude (max deviation from the center position)
# B = Frequency (how fast the oscillation occurs)
# C = Phase Shift (horizontal shift to start the wave at a different point)
# D = Vertical Shift (center position around which the joint oscillates)
# This means each joint would require 4 parameters, leading to a total of 96 parameters (24 joints * 4 parameters each).
# If we set B as a constant value for all joints to maintain a uniform gait speed, then we can reduce the number of parameters per joint to 3 (A, C, D).
# A chromosome would there be a list of 72 floating-point numbers.
# The GA would then optimize these parameters to find the best gait for the spider model.
