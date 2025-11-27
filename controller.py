import numpy as np
from numpy.typing import ArrayLike
from racetrack import RaceTrack

dt = 0.1
V_KP = 12
STEER_KP = 16
STEER_KI = 0
STEER_KD = 0.6
LOOKAHEAD = 25
AHEAD_FOR_SPEED = 15   # how far ahead to check curvature for braking
BLEND = 0.5 # how much of the raceline to use vs the centerline

prevSteerErr = 0
steerErrInt = 0
prevYaw = 0
DERIV_TAU = 0.15
I_MIN = -0.2
I_MAX = 0.2
prevYawRateFiltered = 0.0

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

def steeringEffort(path: np.ndarray, width: np.ndarray, center_idx: int, window: int = 10) -> float:
    """
    Compute a windowed steering effort metric, approximating how much the wheel
    must turn per unit distance along the path.
    """
    half_win = window // 2
    start_idx = max(center_idx - half_win, 1)  # need p1 for first segment
    end_idx = min(center_idx + half_win, len(path) - 3)  # need p3 for last segment

    efforts = []
    distances = []

    for i in range(start_idx, end_idx + 1, 2):
        p1, p2, p3 = path[i-2], path[i], path[i+2]

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
            
        theta = np.tan(theta)
        theta = theta / width[i] * 10

        efforts.append(theta * np.sqrt(norm1))
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
    avg_effort = np.max(delta_effort_per_unit)
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
        
    width = np.linalg.norm(racetrack.right_boundary - racetrack.left_boundary, axis=1)
    
    dists = np.linalg.norm(path - pos, axis=1)
    nearestIndex = int(np.argmin(dists))
    speedLookaheadIndex = getLookaheadPointIndex(path, nearestIndex, LOOKAHEAD)
    
    curv_future = 4 * steeringEffort(path, width, speedLookaheadIndex + AHEAD_FOR_SPEED, 40)
    speed = np.clip(15 + 8 / max(abs(curv_future), 0.001), parameters[2], parameters[5])  # max velocity
    
    lookahead_pen = (100 - speed) / 8
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


def lower_controller(state, desired, parameters):
    global prevSteerErr, steerErrInt, prevYaw, prevYawRateFiltered

    yaw = state[2]
    yaw_des = desired[0]

    # error
    steerErr = (yaw_des - yaw + np.pi) % (2*np.pi) - np.pi

    # derivative ON MEASUREMENT, not error
    rawYawRate = (yaw - prevYaw) / dt
    prevYaw = yaw

    # low-pass filter
    alpha = dt / (DERIV_TAU + dt)   # e.g. DERIV_TAU = 0.15
    yawRateFiltered = (1 - alpha)*prevYawRateFiltered + alpha*rawYawRate
    prevYawRateFiltered = yawRateFiltered

    # PID (P + small filtered D)
    P = STEER_KP * steerErr
    D = - STEER_KD * yawRateFiltered   # minus sign for damping

    # conditional integral: only when not saturated
    tentative = P + D
    if parameters[7] < tentative < parameters[9]:
        steerErrInt += steerErr * dt
    steerErrInt = np.clip(steerErrInt, I_MIN, I_MAX)
    I = STEER_KI * steerErrInt

    steerRate = P + I + D
    steerRate = np.clip(steerRate, parameters[7], parameters[9])

    accel = V_KP * (desired[1] - state[3])
    accel = np.clip(accel, parameters[8], parameters[10])
    return np.array([steerRate, accel])
