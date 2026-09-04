import numpy as np
from numpy.typing import NDArray

#def rms_cross_track_error(data, waypoints):
#    for i in range(len(path) - 1):

def current_path_error(path, actual_location: NDArray):
    error = 1000
    projected_point = np.array([0, 0])

    for i in range(len(path) - 1):
        p1 = path.iloc[i]
        p2 = path.iloc[i + 1]
        p1 = p1[["x", "y"]].to_numpy()
        p2 = p2[["x", "y"]].to_numpy()
        projected_point = segment_projection(actual_location, p1, p2)
        distance_to_path = distance_between_points(projected_point, actual_location)
        if distance_to_path < error:
            error = distance_to_path
            final_projection = projected_point
    return error, final_projection


def segment_projection(q, p1, p2):
    #all inputs shall be two element np.arrays
    u = p2 - p1
    v = q - p1
    projection_parameter = (u @ v) / (np.linalg.norm(u)**2)
    if projection_parameter <= 0:
        projected_point =  p1
    elif projection_parameter >= 1:
        projected_point = p2
    else:
        projected_point = p1 + projection_parameter * u
    return projected_point

def distance_between_points(p1, p2):
    segment = p2 - p1
    return np.linalg.norm(segment)
