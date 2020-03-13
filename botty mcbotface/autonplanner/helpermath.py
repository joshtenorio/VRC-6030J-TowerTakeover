"""
  @file autonplanner/helpermath.py
"""
import math


def vector_magnitude(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))


def vector_normalize(x1, y1, x2, y2):

    output_x = float(x2 - x1) / vector_magnitude(x1, y1, x2, y2)
    output_y = float(y2 - y1) / vector_magnitude(x1, y1, x2, y2)
    output = [output_x, output_y]
    return output
