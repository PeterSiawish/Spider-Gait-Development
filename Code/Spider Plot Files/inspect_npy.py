import numpy as np
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
npy_files = [f for f in os.listdir(script_dir) if f.endswith(".npy")]
gait_file = npy_files[0]

gait_path = os.path.join(script_dir, gait_file)

data = np.load(gait_path)

# This file is to simply analyze the best gait produced per GA.
print(type(data))
print(data.shape)
print(data[0])
