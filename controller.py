import numpy as np
from numpy.typing import ArrayLike
from racetrack import RaceTrack

dt = 0.1
V_KP = 15
STEER_KP = 12
STEER_KI = 16
STEER_KD = 0.2
LOOKAHEAD = 25
AHEAD_FOR_SPEED = 15   # how far ahead to check curvature for braking
BLEND = 0.5 # how much of the raceline to use vs the centerline

prevSteerErr = 0
steerErrInt = 0

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

def average_curvature(path: np.ndarray, center_idx: int, window: int = 10) -> float:
    """
    Compute a windowed steering effort metric, approximating how much the wheel
    must turn per unit distance along the path.
    """
    half_win = window // 2
    start_idx = max(center_idx - half_win, 1)  # need p1 for first segment
    end_idx = min(center_idx + half_win, len(path) - 2)  # need p3 for last segment

    efforts = []
    distances = []

    for i in range(start_idx, end_idx + 1):
        p1, p2, p3 = path[i-1], path[i], path[i+1]

        v1 = p2 - p1
        v2 = p3 - p2
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        if norm1 < 1e-6 or norm2 < 1e-6:
            continue

        # angle between vectors
        cos_theta = np.clip(np.dot(v1, v2) / (norm1 * norm2), -1.0, 1.0)
        theta = np.arccos(cos_theta)

        # signed version to capture left/right turns
        cross = np.cross(v1, v2)
        if cross < 0:
            theta = -theta

        efforts.append(theta)
        # distance along the path corresponding to this angle
        distances.append(norm1)

    if len(efforts) < 2:
        return 0.0

    efforts = np.array(efforts)
    distances = np.array(distances)

    # steering change between consecutive points
    delta_effort = np.diff(efforts)
    delta_effort = (delta_effort + np.pi) % (2*np.pi) - np.pi

    # weight by average distance between consecutive segments
    delta_s = (distances[1:] + distances[:-1]) / 2
    delta_effort_per_unit = np.abs(delta_effort) / np.maximum(delta_s, 1e-6)

    # take top-k largest per-unit steering changes
    topk = np.sort(delta_effort_per_unit)[-3:]
    avg_effort = np.mean(topk)
    return avg_effort

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
    speedLookaheadIndex = getLookaheadPointIndex(path, nearestIndex, LOOKAHEAD)
    
    curv_future = 2 * average_curvature(path, speedLookaheadIndex + AHEAD_FOR_SPEED, 25)
    speed = min(np.sqrt(45 / max(abs(curv_future), 0.001)), parameters[5])  # max velocity
    
    lookahead_pen = int(speed / 30)
    lookaheadIndex = getLookaheadPointIndex(path, nearestIndex, LOOKAHEAD - lookahead_pen)  
    
    vec = path[lookaheadIndex] - pos
    desHead = np.arctan2(vec[1], vec[0])
    headErr = (desHead - heading + np.pi) % (2 * np.pi) - np.pi

    L = parameters[0]  # wheelbase
    lookaheadDist = np.linalg.norm(vec)
    curv = 2 * (np.sin(headErr) / max(lookaheadDist, 0.001))

    steer = np.arctan(L * curv)
    steer = np.clip(steer, parameters[1], parameters[4])  # min/max steering angle

    return np.array([steer, speed], dtype=float)


def lower_controller(state: ArrayLike, desired: ArrayLike, parameters: ArrayLike) -> ArrayLike:
    global prevSteerErr, steerErrInt

    state = np.asarray(state, dtype=float)
    desired = np.asarray(desired, dtype=float)
    parameters = np.asarray(parameters, dtype=float)

    # Steering error
    steerErr = (desired[0] - state[2] + np.pi) % (2 * np.pi) - np.pi
    steerErrRate = (steerErr - prevSteerErr) / dt
    steerErrInt += steerErr * dt  # accumulate integral
    prevSteerErr = steerErr

    # PID for steering
    steerRate = (
        STEER_KP * steerErr +
        STEER_KI * steerErrInt +
        STEER_KD * steerErrRate
    )
    steerRate = np.clip(steerRate, parameters[7], parameters[9])

    # Speed control (unchanged)
    accel = V_KP * (desired[1] - state[3])
    accel = np.clip(accel, parameters[8], parameters[10])

    return np.array([steerRate, accel], dtype=float)