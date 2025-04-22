import numpy as np


def choose_heuristic(he, start, end) -> int:
    if he == "m":
        return manhattan(start, end)
    elif he == "c":
        return chebyshev(start, end)
    elif he == "e":
        return eucledian(start, end)
    elif he == "b":
        return blind(start, end)


def manhattan(start, goal) -> int:
    return abs(start[0]-goal[0]) + abs(start[1]-goal[1])


def chebyshev(start, goal) -> int:
    return max(abs(start[0]-goal[0]), abs(start[1]-goal[1]))


def eucledian(start, goal) -> int:
    return np.sqrt(((abs(start[0]-goal[0]))**2)+(abs(start[1]-goal[1])**2))


def blind(start, goal) -> int:
    return 0
