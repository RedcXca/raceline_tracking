import numpy as np
from numpy.typing import ArrayLike
from racetrack import RaceTrack

dt = 0.1
V_KP = 15
STEER_KP = 12
STEER_KD = 0.01
LOOKAHEAD = 30
BLEND = 0.5 # how much of the raceline to use vs the centerline

prevSteerErr = 0

def resample(path: np.ndarray, n: int) -> np.ndarray:
    # resample path to have exactly n points using linear interpolation
    if len(path) == n:
        return path
    indices = np.linspace(0, len(path) - 1, n)
    x = np.interp(indices, np.arange(len(path)), path[:, 0])
    y = np.interp(indices, np.arange(len(path)), path[:, 1])
    return np.column_stack((x, y))


def getLookaheadPointIndex(path: np.ndarray, startIndex: int, distance: float) -> int:
    # walk along path from startIndex until we've traveled 'distance' meters
    # returns the index of the point we end up at
    pathLen = len(path)
    traveled = 0.0
    currentIndex = startIndex
    iterations = 0
    
    while traveled < distance and iterations < min(pathLen, 1000):
        nextIndex = (currentIndex + 1) % pathLen
        segmentLen = np.linalg.norm(path[nextIndex] - path[currentIndex])
        if segmentLen == 0:
            break
        traveled += segmentLen
        currentIndex = nextIndex
        iterations += 1
    
    return currentIndex


def controller(state: ArrayLike, parameters: ArrayLike, racetrack: RaceTrack) -> ArrayLike:
    state = np.asarray(state, dtype=float)
    parameters = np.asarray(parameters, dtype=float)

    pos = state[:2]
    heading = state[4]

    if hasattr(racetrack, 'raceline') and racetrack.raceline is not None:
        race = racetrack.raceline
        center = racetrack.centerline
        if len(race) != len(center):
            center = resample(center, len(race))
        path = BLEND * race + (1 - BLEND) * center
    else:
        path = racetrack.centerline
    
    dists = np.linalg.norm(path - pos, axis=1)
    nearestIndex = int(np.argmin(dists))
    lookaheadIndex = getLookaheadPointIndex(path, nearestIndex, LOOKAHEAD)
    
    vec = path[lookaheadIndex] - pos
    desHead = np.arctan2(vec[1], vec[0])
    headErr = (desHead - heading + np.pi) % (2 * np.pi) - np.pi

    L = parameters[0]  # wheelbase
    lookaheadDist = np.linalg.norm(vec)
    curv = 2 * np.sin(headErr) / max(lookaheadDist, 0.001)
    
    steer = np.arctan(L * curv)
    steer = np.clip(steer, parameters[1], parameters[4])  # min/max steering angle

    speed = min(np.sqrt(45 / max(abs(curv), 0.001)), parameters[5])  # max velocity

    return np.array([steer, speed], dtype=float)


def lower_controller(state: ArrayLike, desired: ArrayLike, parameters: ArrayLike) -> ArrayLike:
    global prevSteerErr

    state = np.asarray(state, dtype=float)
    desired = np.asarray(desired, dtype=float)
    parameters = np.asarray(parameters, dtype=float)

    # desired[0] = desired steering angle, state[2] = current steering angle
    steerErr = (desired[0] - state[2] + np.pi) % (2 * np.pi) - np.pi
    steerErrRate = (steerErr - prevSteerErr) / dt
    prevSteerErr = steerErr

    steerRate = STEER_KP * steerErr + STEER_KD * steerErrRate
    steerRate = np.clip(steerRate, parameters[7], parameters[9])

    # desired[1] = desired speed, state[3] = current speed
    accel = V_KP * (desired[1] - state[3])
    accel = np.clip(accel, parameters[8], parameters[10])

    return np.array([steerRate, accel], dtype=float)
