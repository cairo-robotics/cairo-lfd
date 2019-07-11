import numpy as np


def perimeter_heuristic():
    pass


def over_under_heuristic():
    pass


def orientation_heuristic():
    pass


def height_heuristic(mean, covar):
    '''
    Purpose: Finds upper and lower bound of gaussian distribution for a given segment
    and then chooses and returns 5 linearly spaced points within that boundary to use 
    as a heuristic for the height constraint
    '''

    # find the rotation matrix and radii of the axes
    U, s, rotation = linalg.svd(covar)
    radii = np.sqrt(s)

    # now carry on with EOL's answer
    u = np.linspace(0.0, 2.0 * np.pi, 100)
    v = np.linspace(0.0, np.pi, 100)
    x = radii[0] * np.outer(np.cos(u), np.sin(v))
    y = radii[1] * np.outer(np.sin(u), np.sin(v))
    z = radii[2] * np.outer(np.ones_like(u), np.cos(v))

    for i in range(len(x)):
        for j in range(len(x)):
            [x[i, j], y[i, j], z[i, j]] = np.dot(
                [x[i, j], y[i, j], z[i, j]], rotation) + mean

    # get index of upper and lower bound
    maxz = -100
    maxzindouter = 0
    maxzindinner = 0

    minz = 100
    minzindouter = 0
    minzindinner = 0

    for i in range(len(z)):
        temp = z[i].tolist()
        if max(temp) > maxz:
            maxz = max(temp)
            maxzindouter = i
            maxzindinner = temp.index(max(temp))
        if min(temp) < minz:
            minz = min(temp)
            minzindouter = i
            minzindinner = temp.index(min(temp))

    # use bounds to find 5 acceptable heights
    heights = np.linspace(minz, maxz, 5)
    # print(heights)

    return heights
