import numpy as np
from scipy.linalg import expm

def compute_linearization(v, theta):
    A = np.array([
        [0, 0, -v * np.sin(theta)],
        [0, 0,  v * np.cos(theta)],
        [0, 0, 0]
    ])

    B = np.array([
        [np.cos(theta), 0],
        [np.sin(theta), 0],
        [0, 1]
    ])

    return A, B

# Discretize
def discretize_linear_model(A, B, dt):
    n = A.shape[0]
    m = B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    M_exp = expm(M * dt)
    A_d = M_exp[:n, :n]
    B_d = M_exp[:n, n:]
    return A_d, B_d

def generate_square_ref(total_steps, dt, waypoints, speed=0.5):
    traj = []
    for i in range(len(waypoints)-1):
        p0 = waypoints[i]
        p1 = waypoints[i+1]
        d = np.linalg.norm(p1 - p0)
        steps = max(1, int(d / (speed * dt)))
        for s in range(steps):
            alpha = s / steps
            pos = (1 - alpha) * p0 + alpha * p1
            angle = np.arctan2(p1[1] - p0[1], p1[0] - p0[0])
            traj.append(np.array([pos[0], pos[1], angle]))
        traj.append(np.array([p1[0], p1[1], np.arctan2(p1[1]-p0[1], p1[0]-p0[0])]))
    print("First few points in trajectory:")
    for i in range(5):
        print(traj[i])
    return np.array(traj)