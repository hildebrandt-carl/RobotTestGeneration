import math
import numpy as np
from scipy.linalg import block_diag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Given a time T and waypoints w
# Assign a time each waypoint should be met scaled by distance
def arrangeT(w, T):

    # Subtract each waypoint from previous waypoint
    x = w[:, 1:] - w[:, 0:-1]
    # Get the distance between each consecutive waypoint
    dist = np.sqrt(np.sum(np.square(x), axis=0))
    # Calculate time over total distance
    k = T/sum(dist)
    # Starting at time 0 add each of the distance * delta time
    ts = np.hstack((np.array([0]), np.cumsum(dist*k)))
    # Return the time
    return ts

def computeQ(n, r, t1, t2):
    T = np.zeros(((n-r)*2+1, 1))
    for i in range(0, (n-r)*2+1):
        T[i] = t2**(i+1) - t1**(i+1)

    Q = np.zeros((n+1, n+1))
    for i in range(r, n + 1):
        for j in range(i, n + 1):
            k1 = i - r
            k2 = j - r
            k = k1 + k2 + 1
            Q[i, j] = np.prod(np.arange(k1 + 1, k1 + r + 1)) * np.prod(np.arange(k2 + 1, k2 + r + 1))/k * T[k-1]
            Q[j, i] = Q[i, j]
    return Q

def minimum_snap_single_axis_close_form(wayp, ts, n_order, v0, a0, v1, a1):

    n_coef = n_order+1
    n_poly = len(wayp)-1
    polys = 0

    Q_all = np.array([])
    for i in range(0, n_poly):
        Q_all = block_diag(Q_all, computeQ(n_order, 3, ts[i], ts[i+1]))

    # Remove the first row of the Q_all array
    Q_all = Q_all[1:, :]

    # compute Tk
    tk = np.zeros((n_poly + 1, n_coef))
    for i in range(0, n_coef):
        tk[:, i] = ts[:] ** i

    n_continuous = 3
    A = np.zeros((n_continuous * 2 * n_poly, n_coef * n_poly))
    for i in range(0, n_poly):
        for j in range(0, n_continuous):
            for k in range(j, n_coef):
                if k == j:
                    t1 = 1
                    t2 = 1
                else:
                    t1 = tk[i, k - j]
                    t2 = tk[i + 1, k - j]
                a = np.prod(np.arange(k - j + 1, k+1)) * t1
                b = np.prod(np.arange(k - j + 1, k+1)) * t2
                index11 = n_continuous * 2 * (i) + j
                index12 = n_coef * (i) + k
                A[index11, index12] = a
                index21 = n_continuous * 2 * (i) + n_continuous + j
                index22 = n_coef * (i) + k
                A[index21, index22] = b

    # compute M
    M = np.zeros((n_poly * 2 * n_continuous, n_continuous * (n_poly + 1)))
    for i in range(1, n_poly * 2 + 1):
        j = math.floor(i / 2) + 1
        rbeg = n_continuous * (i - 1)
        cbeg = n_continuous * (j - 1)
        M[rbeg:rbeg + n_continuous, cbeg:cbeg + n_continuous] = np.eye(n_continuous)

    # compute C
    num_d = n_continuous * (n_poly + 1)
    C = np.eye(num_d)
    df = np.concatenate([wayp, np.array([v0, a0, v1, a1])])
    fix_idx = np.concatenate([np.arange(0, num_d, 3), np.array([1, 2, num_d - 2, num_d-1])])
    free_idx = np.setdiff1d(np.arange(0, num_d), fix_idx)
    C = np.hstack([C[:, fix_idx], C[:, free_idx]])

    AiMC = np.dot(np.dot(np.linalg.inv(A), M), C)
    R = np.dot(np.dot(AiMC.T, Q_all), AiMC)

    n_fix = len(fix_idx)
    Rff = R[0:n_fix, 0:n_fix]
    Rpp = R[n_fix:, n_fix:]
    Rfp = R[0:n_fix, n_fix:]
    Rpf = R[n_fix:, 0:n_fix]

    dp = np.dot(np.dot((-1 * np.linalg.inv(Rpp)), Rfp.T), df)

    p = np.dot(AiMC, np.concatenate([df, dp]))

    polys = p.reshape(n_poly, n_coef).T

    return polys

def poly_val(poly, t, r):
    val = 0
    n = len(poly)-1
    if r <= 0:
        for ind in range(0, n + 1):
            val = val + poly[ind] * t**ind
    else:
        for ind in range(r, n):
            a = poly[ind+1] * np.prod(np.arange(ind-r+1, ind)) * t**(ind-r)
            val = val + a
    return val

def polys_vals(polys, ts, tt, r):
    idx = 0
    N = len(tt)
    vals = np.zeros((1, N)).reshape(N)
    for i in range(0, N):
        t = tt[i]
        if t < ts[idx]:
            vals[i] = 0
        else:
            while idx < len(ts) and t > ts[idx+1] + 0.0001:
                idx = idx+1
            vals[i] = poly_val(polys[:, idx], t, r)

    return vals

waypoints = np.array([[0, 0, 0],
                     [5, 2, 5],
                     [2, 2, 2],
                     [7, 3, 2],
                     [3, 3, 4]]).T

# # Example of actual waypoints
waypoints = np.array([[0.1, -0.1, 0.1],
                    [7.01746807, -0.47438195, 12.33942555],
                    [17.69236745, -12.35556763, 17.97738286],
                    [23.97354414, -9.67153598, 23.77979945],
                    [22.40996379, -8.67317643, 23.06455235],
                    [24.40126482, -13.25209284, 15.89520135],
                    [21.31856002, -16.83695051, 20.21547731],
                    [21.52024023, -17.25974625, 26.19989471],
                    [12.9570001, -18.67680441, 28.17673884],
                    [15., -15., 15.]]).T


waypoints = np.array([[  0.1,  -0.1,   0.1],
  [  12.97,-1.126 , 5.988],
  [  28.64 , -10.39  , 23.62],
  [  25.95 , -25.60 ,  24.18],
  [  20.32 , -29.76 ,  5.157],
  [  15.52 , -26.39 ,  0.02011],
  [  13.26 , -14.81 ,  7.603],
  [  20.20 , -4.138 ,  27.77],
  [  18.28 , -1.145,   26.50],
  [  15.00 , -15.00 ,  15.00]]).T


v0 = np.array([0, 0, 0])
a0 = np.array([0, 0, 0])
v1 = np.array([0, 0, 0])
a1 = np.array([0, 0, 0])

T = 10
ts = arrangeT(waypoints, T)

n_order = 5

polys_x = minimum_snap_single_axis_close_form(waypoints[0, :], ts, n_order, v0[0], a0[0], v1[0], a1[0])
polys_y = minimum_snap_single_axis_close_form(waypoints[1, :], ts, n_order, v0[1], a0[1], v1[1], a1[1])
polys_z = minimum_snap_single_axis_close_form(waypoints[2, :], ts, n_order, v0[2], a0[2], v1[2], a1[2])

# Build the trajectories
xx = np.array([])
yy = np.array([])
zz = np.array([])
waypoint_index = np.array([])
for i in range(0, polys_x.shape[1]):
    tt = np.arange(ts[i], ts[i+1], 0.01)
    xx = np.concatenate([xx, polys_vals(polys_x, ts, tt, 0)])
    yy = np.concatenate([yy, polys_vals(polys_y, ts, tt, 0)])
    zz = np.concatenate([zz, polys_vals(polys_z, ts, tt, 0)])
    wy = np.zeros((1, len(tt))).reshape(-1)
    wy[0] = 1
    waypoint_index = np.concatenate([waypoint_index, wy])

xx = np.concatenate([xx, np.array([waypoints[0][-1]])])
yy = np.concatenate([yy, np.array([waypoints[1][-1]])])
zz = np.concatenate([zz, np.array([waypoints[2][-1]])])
waypoint_index = np.concatenate([waypoint_index, np.array([1])])

fig = plt.figure(1)
ax = fig.gca(projection='3d')
ax.scatter(waypoints[0, :], waypoints[1, :], waypoints[2, :], label='Waypoints')
ax.plot(waypoints[0, :], waypoints[1, :], waypoints[2, :], linestyle='--', label='Original Line')
ax.plot(xx, yy, zz, label='Minimum Snap Line')
ax.legend()
plt.show()