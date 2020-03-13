"""
  @file autonplanner/main.py
"""
import pathgenerator as penguin
import helpermath as puppy
import math
import numpy as np
import matplotlib.pyplot as plt
import pylab

path = [[0.0, 0.0], [0.0, 5.0], [5.0, 5.0], [5.0, 10.0]]
new_path = penguin.inject_points(path)

b = 0.3
a = 1 - b
final_path = penguin.smooth_path_laplacian(new_path, 77)
print(final_path)

processed_data = np.array(final_path).astype("float")

# data[row][column] -> row[n] is nth point -> column[0] = x, column[1] = y
first_point = processed_data[0]
second_point = processed_data[1]
x1 = first_point[0]
y1 = first_point[1]
x2 = second_point[0]
y2 = second_point[1]

# plot data and save as png
plt.plot(processed_data[:, 0], processed_data[:, 1])
plt.arrow(x1, y1, 0, 0, width=0.09, head_width=0.5)

plt.show()
