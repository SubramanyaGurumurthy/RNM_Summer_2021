import numpy as np

filename = "calibration_poses"

data = np.loadtxt(filename, delimiter=",")

print(data)