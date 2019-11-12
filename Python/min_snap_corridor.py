import matlab.engine
import numpy as np
from scipy.linalg import block_diag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from multiprocessing import Pool


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


def calc_tvec(t, n_order, r):
    tvec = np.zeros((1, n_order+1))
    for ij in range(r+1, n_order+2):
        tvec[0, ij-1] = np.prod(np.arange(ij-r, ij))*t**(ij-r-1)
    return np.array(tvec).reshape(-1)


def minimum_snap_single_axis_corridor(args):

    waypts, ts, n_order, v0, a0, ve, ae, corridor_r, orig_waypt_indx = args

    orig_waypt = waypts[orig_waypt_indx.astype(bool)]

    p0 = waypts[0]
    pe = waypts[-1]

    n_coef = n_order+1
    n_poly = len(waypts)-1

    Q_all = np.array([])
    for i in range(0, n_poly):
        Q_all = block_diag(Q_all, computeQ(n_order, 3, ts[i], ts[i+1]))

    # Remove the first row of the Q_all array
    Q_all = Q_all[1:, :]

    b_all = np.zeros((Q_all.shape[0], 1))

    Aeq = np.zeros((3 * n_poly + 3 + len(orig_waypt), n_coef * n_poly))
    beq = np.zeros((3 * n_poly + 3 + len(orig_waypt), 1))

    Aeq[0: 3, 0: n_coef] = np.array([calc_tvec(ts[0], n_order, 0),
                                     calc_tvec(ts[0], n_order, 1),
                                     calc_tvec(ts[0], n_order, 2)])

    Aeq[3: 6, n_coef * (n_poly - 1):n_coef * n_poly] = np.array([calc_tvec(ts[-1], n_order, 0),
                                                                 calc_tvec(ts[-1], n_order, 1),
                                                                 calc_tvec(ts[-1], n_order, 2)])

    beq[0:6, 0] = np.array([p0, v0, a0, pe, ve, ae]).T
    neq = 5

    # continuous constraints((n_poly - 1) * 3 equations)
    for i in range(0, n_poly - 1):
        tvec_p = calc_tvec(ts[i + 1], n_order, 0)
        tvec_v = calc_tvec(ts[i + 1], n_order, 1)
        tvec_a = calc_tvec(ts[i + 1], n_order, 2)
        neq = neq + 1
        Aeq[neq, n_coef * i:n_coef * (i + 2)] = np.concatenate([tvec_p, -tvec_p])
        neq = neq + 1
        Aeq[neq, n_coef * i: n_coef * (i + 2)] = np.concatenate([tvec_v, -tvec_v])
        neq = neq + 1
        Aeq[neq, n_coef * i: n_coef * (i + 2)] = np.concatenate([tvec_a, -tvec_a])

    # Add constraint to go through waypoints
    i = 0
    for ind in orig_waypt_indx:
        if i >= len(orig_waypt_indx) -1:
            break
        if ind == 1:
            neq = neq + 1
            tvec_p = calc_tvec(ts[i + 1], n_order, 0)
            Aeq[neq, n_coef * i: n_coef * (i + 1)] = np.array(tvec_p)
            beq[neq] = waypts[i]
        i += 1

    # corridor constraints(n_ploy - 1 iequations)
    Aieq = np.zeros((2 * (n_poly - 1), n_coef * n_poly))
    bieq = np.zeros((2 * (n_poly - 1), 1))

    # for i in range(0, n_poly-1):
    #     tvec_p = calc_tvec(ts[i + 1], n_order, 0)
    #     i1 = 2 * i
    #     i2 = 2 * (i + 1)
    #     i3 = n_coef * (i + 1)
    #     i4 = n_coef * (i + 2)
    #     Aieq[i1:i2, i3:i4] = np.array([tvec_p, -tvec_p])
    #     bieq[i1:i2] = np.array([waypts[i+1]+corridor_r, corridor_r-waypts[i+1]]).reshape(2, 1)

    for i in range(0, n_poly-1):
        tvec_p = calc_tvec(ts[i + 1], n_order, 0)
        i1 = 2 * i
        i2 = 2 * (i + 1)
        i3 = n_coef * (i + 1)
        i4 = n_coef * (i + 2)
        Aieq[i1:i2, i3:i4] = np.array([tvec_p, -tvec_p])
        bieq[i1:i2] = np.array([waypts[i] + corridor_r, corridor_r - waypts[i]]).reshape(2, 1)

    eng = matlab.engine.start_matlab()
    Q_all_m = matlab.double(list(Q_all.tolist()))
    b_all_m = matlab.double(list(b_all.tolist()))
    Aieq_m = matlab.double(list(Aieq.tolist()))
    bieq_m = matlab.double(list(bieq.tolist()))
    Aeq_m = matlab.double(list(Aeq.tolist()))
    beq_m = matlab.double(list(beq.tolist()))
    blank = matlab.double([])

    options = eng.optimoptions('quadprog', 'MaxIterations', 25)
    p = eng.quadprog(Q_all_m, b_all_m, Aieq_m, bieq_m, Aeq_m, beq_m, blank, blank, blank, options)

    # # p = quadprog(Q_all, b_all, Aieq, bieq, Aeq, beq)
    # sol = cvxopt.solvers.qp(cvxopt.matrix(Q_all), cvxopt.matrix(b_all), cvxopt.matrix(Aieq), cvxopt.matrix(bieq), cvxopt.matrix(Aeq), cvxopt.matrix(beq))
    # p = np.array(cvxopt.matrix(sol['x']))

    np_p = np.array(p._data.tolist())
    np_p = np_p.reshape(p.size).transpose()

    polys = np_p.reshape(n_poly, n_coef).T

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
T = 5
n_order = 5

# Re-sample mid points
r = 0.2
step = r
new_waypts = np.array(waypoints[:, 0]).reshape(3, 1)
for i in range(0, waypoints.shape[1]-1):
    x1 = waypoints[0, i]
    y1 = waypoints[1, i]
    z1 = waypoints[2, i]
    x2 = waypoints[0, i+1]
    y2 = waypoints[1, i+1]
    z2 = waypoints[2, i+1]
    n = int(np.ceil((np.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2))/step)+1)
    sample_pts = np.vstack([np.linspace(x1, x2, n), np.linspace(y1, y2, n), np.linspace(z1, z2, n)])
    new_waypts = np.hstack([new_waypts, sample_pts[:, 2:]])

# Mark the original waypoints
original_waypoint_index = np.zeros((new_waypts.shape[1], 1))
for i in range(0, new_waypts.shape[1]):
    for j in range(0, waypoints.shape[1]):
        if (new_waypts[:, i] == waypoints[:, j]).all():
            original_waypoint_index[i] = 1

original_waypoint_index = original_waypoint_index.reshape(-1)

ts = arrangeT(new_waypts, T)

input1 = new_waypts[0, :], ts, n_order,  v0[0], a0[0], v1[0], a1[0], r, original_waypoint_index
input2 = new_waypts[1, :], ts, n_order,  v0[0], a0[0], v1[0], a1[0], r, original_waypoint_index
input3 = new_waypts[2, :], ts, n_order,  v0[0], a0[0], v1[0], a1[0], r, original_waypoint_index

p = Pool(3)
results = p.map(minimum_snap_single_axis_corridor, [input1, input2, input3])

polys_x = results[0]
polys_y = results[1]
polys_z = results[2]

# polys_x = minimum_snap_single_axis_corridor(input1)
# polys_y = minimum_snap_single_axis_corridor(input2)
# polys_z = minimum_snap_single_axis_corridor(input3)

tt = np.arange(0, T, 0.01)
xx = polys_vals(polys_x, ts, tt, 0)
yy = polys_vals(polys_y, ts, tt, 0)
zz = polys_vals(polys_z, ts, tt, 0)

fig = plt.figure(1)
ax = fig.gca(projection='3d')
ax.scatter(waypoints[0, :], waypoints[1, :], waypoints[2, :], label='Waypoints')
ax.plot(waypoints[0, :], waypoints[1, :], waypoints[2, :], linestyle='--', label='Original Line')
ax.plot(xx, yy, zz, label='Minimum Snap Line')
ax.legend()
plt.show()
