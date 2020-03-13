"""
  @file autonplanner/pathgenerator.py
"""
import helpermath as puppy
import math

def inject_points(waypoints, spacing=1):
    new_path = []
    num_line_segments = len(waypoints) - 1

    for x in range(num_line_segments):
        x1 = waypoints[x][0]
        y1 = waypoints[x][1]
        x2 = waypoints[x + 1][0]
        y2 = waypoints[x + 1][1]

        num_points_that_fit = math.ceil(puppy.vector_magnitude(x1, y1, x2, y2) / spacing)

        vector = puppy.vector_normalize(x1, y1, x2, y2)

        vector[0] = vector[0] * spacing
        vector[1] = vector[1] * spacing

        for i in range(num_points_that_fit):
            injected_point = [x1 + vector[0] * float(i), y1 + vector[1] * float(i)]
            new_path.append(injected_point)

    new_path.append(waypoints[len(waypoints) - 1])
    return new_path



def smooth_path_laplacian(waypoints, num):
    new_path = waypoints
    tmp_path = waypoints

    # how many times to smooth the path
    for n in range(num):

        for i in range(1, len(tmp_path) - 1):
            for j in range(len(tmp_path[i])):
                mid_point = (tmp_path[i - 1][j] + tmp_path[i + 1][j]) / 2
                new_path[i][j] = mid_point

        tmp_path = new_path

    return new_path
