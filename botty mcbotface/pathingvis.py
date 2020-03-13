"""
@file pathingvis.py
"""
import os
import json
import csv
import numpy as np
import matplotlib.pyplot as plt
import pylab

# open data/index.json and get the last visualized file's ID
with open("data_index.json") as file:
    last_file_read = json.load(file)['LatestFile']

# figure out which csv file is the newest, and only visualize that data
# WIP
data_dir = os.listdir("data")
newest_data = -1
for file in data_dir:
    pass

# open csv file and read out data into numpy 2D array
csv_data = csv.reader(open("data/debugTestPathnewPathLog.csv", "rt"), delimiter=",")

raw_data = list(csv_data)

processed_data = np.array(raw_data).astype("float")

# data[row][column] -> row[n] is nth point -> column[0] = x, column[1] = y
first_point = processed_data[0]
second_point = processed_data[1]
x1 = first_point[0]
y1 = first_point[1]
x2 = second_point[0]
y2 = second_point[1]

# plot data and save as png
plt.plot(processed_data[:, 0], processed_data[:, 1])
# plt.arrow(x1, y1, 0, 0, width=0.09, head_width=0.5)

# need to add ID at end of filename, like path<ID>.png
pylab.savefig('path.png')
plt.show()
