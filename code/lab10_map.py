import numpy as np
import json
import math


# see https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
def ray_line_intersection(ray_origin, ray_direction, point1, point2):
    # Convert to numpy arrays
    ray_origin = np.array(ray_origin, dtype=np.float)
    ray_direction = np.array([math.cos(ray_direction), math.sin(ray_direction)])
    point1 = np.array(point1, dtype=np.float)
    point2 = np.array(point2, dtype=np.float)

    # Ray-Line Segment Intersection Test in 2D
    v1 = ray_origin - point1
    v2 = point2 - point1
    v3 = np.array([-ray_direction[1], ray_direction[0]])
    denominator = np.dot(v2, v3)
    if denominator == 0:
        return None
    t1 = np.cross(v2, v1) / denominator
    t2 = np.dot(v1, v3) / denominator
    if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
        return [ray_origin + t1 * ray_direction]
    return None


class Map:
    def __init__(self, file_name):
        with open(file_name, 'r') as f:
            self._map = json.load(f)
        # initialize boundaries
        self.bottom_left = [0, 0]
        self.top_right = [3, 3]

    def closest_distance(self, origin, theta):
        result = None
        for line in self._map["lines"]:
            p = ray_line_intersection(origin, theta, line["start"], line["end"])
            if p is not None:
                dist = np.linalg.norm(np.array(p) - np.array(origin))
                if result is None:
                    result = dist
                else:
                    result = min(result, dist)
        return result

