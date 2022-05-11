import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from filterpy.stats import plot_covariance_ellipse
from numpy.linalg import inv
from numpy.random import randn

def plot_line(r, phi, start, end, color='red'):
    if phi > np.pi/4:
        x = np.linspace(start[0],end[0], 10)
        y = (r - x*np.cos(phi)) / np.sin(phi)
    else:
        y = np.linspace(start[1], end[1], 10)
        x = (r - y*np.sin(phi)) / np.cos(phi)
    print(x)
    print(y)
    plt.plot(x, y, color = color)


# Inverse measurement model: Maps measurement to state
# This is used since we need to find some relationship between measurements and the robot pose 
'''def inv_h(x, l):
    r_G = l[0,0] # distance from origin of global coordinate system G to orthogonal projected point on line feature
    phi_G = l[1,0] # Angle between x-axis of G
    _x = x[0,0] # x position of robot
    _y = x[1,0] # y position of robot
    _theta = x[2,0] # robot heading
    r_R = r_G - (_x*np.cos(phi_G) + _y*np.sin(phi_G))
    phi_R = phi_G - _theta
    return np.array([[r_R],
                     [phi_R]])'''
    
def inv_h(x, l):
    r_R = l[0,0]
    phi_R = l[1,0]
    _x = x[0,0]
    _y = x[1,0]
    _theta = x[2,0]
    phi_G = phi_R + _theta
    r_G = r_R + _x*np.cos(phi_G) + _y*np.sin(phi_G)
    return np.array([[r_G],
                     [phi_G]])



# Measurement model for landmark i: Maps landmark i in state to measurement
'''
def h_i(nu, i):
    landmark_index = 3 + i*2
    r_R = nu[landmark_index, 0]
    phi_R = nu[landmark_index+1, 0]
    _x = nu[0,0]
    _y = nu[1,0]
    _theta = nu[2,0]
    r_G = r_R + (_x*np.cos(phi_R + _theta) + _y*np.sin(phi_R + _theta))
    phi_G = phi_R + _theta
    return np.array([[r_G], 
                     [phi_G]]) '''

def convert_measurement_to_robot_frame(nu, y_meas):
    r_G = y_meas[0, 0]
    phi_G = y_meas[1, 0]
    _x = nu[0,0]
    _y = nu[1,0]
    _theta = nu[2,0]
    r_R = r_G - (_x*np.cos(phi_G) + _y*np.sin(phi_G))
    phi_R = phi_G - _theta
    return np.array([[r_R],
                     [phi_R]])


def h_i(nu, i):
    landmark_index = 3 + i*2
    r_G = nu[landmark_index, 0]
    phi_G = nu[landmark_index+1, 0]
    _x = nu[0,0]
    _y = nu[1,0]
    _theta = nu[2,0]
    r_R = r_G - (_x*np.cos(phi_G) + _y*np.sin(phi_G))
    phi_R = phi_G - _theta
    return np.array([[r_R],
                     [phi_R]])

# Measurement model: Stacked h_i
def h(nu):
    num_landmarks = int((np.shape(nu)[0] - 3) / 2)
    meas_model = h_i(nu, 0)
    for i in range(1, num_landmarks):
        meas_model = np.vstack((meas_model, h_i(nu, i)))
    return meas_model

def add_new_landmark_to_state(nu, l):
    return np.block([[nu], 
                     [l]])


'''def Hx(nu, i):
    jacobian_hx = np.zeros((2,3))
    landmark_index = 3 + i*2
    _x = nu[0,0]
    _y = nu[1,0]
    _theta = nu[2,0]
    phi_R = nu[landmark_index+1]
    jacobian_hx[0,0] = np.cos(phi_R + _theta)
    jacobian_hx[0,1] = np.sin(phi_R + _theta)
    jacobian_hx[0,2] = -_x*np.sin(phi_R + _theta) + _y*np.cos(phi_R + _theta)
    jacobian_hx[1,2] = 1
    return jacobian_hx'''

def Hx(nu, i):
    jacobian_hx = np.zeros((2,3))
    landmark_index = 3 + i*2
    phi_G = nu[landmark_index+1,0]
    _x = nu[0,0]
    _y = nu[1,0]
    jacobian_hx[0,0] = -_x*np.cos(phi_G)
    jacobian_hx[0,1] = -_y*np.sin(phi_G)
    jacobian_hx[1,2] = -1
    return jacobian_hx

'''def Hl(nu, i):
    jacobian_hl = np.eye(2,2)
    landmark_index = 3 + i*2
    _x = nu[0,0]
    _y = nu[1,0]
    _theta = nu[2,0]
    phi_R = nu[landmark_index+1]
    jacobian_hl[0,1] = -_x*np.sin(phi_R+_theta) + _y*np.cos(phi_R + _theta)
    return jacobian_hl'''

def Hl(nu, i):
    jacobian_hl = np.eye(2,2)
    landmark_index = 3 + i*2
    phi_G = nu[landmark_index+1,0]
    _x = nu[0,0]
    _y = nu[1,0]
    jacobian_hl[0,1] = _x*np.sin(phi_G) - _y*np.cos(phi_G)
    return jacobian_hl

# Jacobian of stacked measurement model vector
def H(nu):
    num_landmarks = int((np.shape(nu)[0]-3) / 2)
    #matrix = np.block([[Hx(nu, 0), Hl(nu, 0)]])
    block_Hx = Hx(nu, 0)
    block_Hl = Hl(nu, 0)
    for i in range(1, num_landmarks):
        _Hx = Hx(nu, i)
        _Hl = Hl(nu, i)
        block_Hx = np.block([[block_Hx], 
                             [_Hx]])
        block_Hl = np.block([[block_Hl, np.zeros((2*i, 2))],
                             [np.zeros((2, 2*i)), _Hl]])
    
    matrix = np.block([[block_Hx, block_Hl]])
    return matrix


def G(n):
    return np.block([[np.eye(3,3)],
                     [np.zeros((n*2,3))]])

def f(nu, u, dt):
    x = nu[0,0] + dt*u[0]*np.cos(nu[2,0])
    y = nu[1,0] + dt*u[0]*np.sin(nu[2,0])
    theta = nu[2,0] + dt*u[1]
    #r1 = nu[3,0]
    #phi1 = nu[4,0]
    state =  np.array([
        [x],
        [y],
        [theta],
    ])
    if (np.shape(nu)[0] > 3):
        return np.block([[state], 
                         [nu[3:,:]]])
    else:
        return state

def Fx(nu, u, dt):
    matrix = np.eye(3,3)
    matrix[0,2] = -dt*u[0]*np.sin(nu[2, 0])
    matrix[1,2] = dt*u[0]*np.cos(nu[2, 0])
    return matrix


def F(nu, u, dt):
    n = int((np.shape(nu)[0] - 3) / 2)
    matrix = np.block([[Fx(nu, u, dt), np.zeros((3,n*2))],
                       [np.zeros((n*2,3)), np.eye(n*2,n*2)]])
    return matrix

def add_new_landmark_covariance(R, new_cov):
    num_landmarks = int(np.shape(R)[0] / 2)
    return np.block([[R, np.zeros((2*num_landmarks,2))],
                     [np.zeros((2, num_landmarks*2)), new_cov]])

def Gx(nu, y_R):
    phi_R = y_R[1,0]
    _x = nu[0,0]
    _y = nu[1,0]
    _theta = nu[2,0]
    inv_meas_mod_jacobian_x = np.array([[np.cos(phi_R + _theta), np.sin(phi_R + _theta), -_x*np.sin(phi_R + _theta) + _y*np.cos(phi_R + _theta)],
                                        [0, 0, 1]])
    return inv_meas_mod_jacobian_x

def Gy(nu, y_R):
    phi_R = y_R[1,0]
    _x = nu[0,0]
    _y = nu[1,0]
    _theta = nu[2,0]
    inv_meas_mod_jacobian_y = np.array([[1, -_x*np.sin(phi_R + _theta) + _y*np.cos(phi_R + _theta)],
                                        [0, 1]])
    return inv_meas_mod_jacobian_y 

def update_measurement_noise_for_observation(prev_R, meas_noise, i):
    updated_R = prev_R
    updated_R[i*2:i*2+2, i*2:i*2+2] = meas_noise
    return updated_R

def predict(nu, P, u, dt, num_landmarks):
    nu_ = f(nu, u, dt) # Predict state
    P_ = F(nu, u, dt) @ P @ F(nu, u, dt).T + G(num_landmarks) @ Q @ G(num_landmarks).T # Predict covariance
    return nu_, P_

def add_new_landmark(nu, P, R, y_meas, R_meas):
    #y0 = np.array([[10], [0.0]])              # Line feature measurement (NB: global frame is the "sensor frame")
    #R_meas = np.eye(2,2)*np.array([[0.1],[0.001]]) # Measurement noise for feature 0

    # Map landmark to state using inverse measurement model
    l = inv_h(nu, y_meas)
    nu_ = add_new_landmark_to_state(nu, y_meas) # Append new landmark to state vector

    # Initial landmark covariance
    P_ll = Gx(nu_, y_meas) @ P[0:3, 0:3] @ Gx(nu_, y_meas).T + Gy(nu_, y_meas) @ R_meas @ Gy(nu_, y_meas).T
    # Initial landmark cross-covariance
    P_lx = Gx(nu_, y_meas) @ P[0:3, :]
    # Append initial landmark covariance matrix to full-state covariance matrix
    P_ = np.block([[P, P_lx.T], 
                [P_lx, P_ll]])
    
    if np.shape(nu)[0]-3 > 1:
        R = add_new_landmark_covariance(R, R_meas)
        return nu_, P_, R
    else:
        # This line is the first one observed
        return nu_, P_, R_meas

def residual(a, b):
    """ compute residual (a-b) between measurements containing 
    [range, bearing]. Bearing is normalized to [-pi, pi)"""
    y = a - b
    y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[1] > np.pi:             # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y

def update(nu, P, R, landmark_index, y_meas, R_meas): # We have observed a line feature already part of the map
    i = landmark_index
    
    R_ = update_measurement_noise_for_observation(R, R_meas, i)

    P_xx = P[0:3,0:3]
    P_xli = P[0:3,3+i*2:3+i*2+2]

    P_lli = P[3+i*2:3+i*2+2, 3+i*2:3+i*2+2]



    # computing the innovation is sparse: only take care of the robot state, the concerned landmark and the robot-landmark i covariances
    Z = np.block([[Hx(nu,i), Hl(nu, i)]]) @ np.block([[P_xx, P_xli],
                                                    [P_xli.T, P_lli]]) @ np.block([[Hx(nu, i).T], 
                                                                                    [Hl(nu, i).T]]) + R_[i*2:i*2+2, i*2:i*2+2]

    P_mr = P[3:,0:3]
    P_mli = P[3:, 3+i*2:3+i*2+2]

    # Compute Kalman gain
    K = np.block([[P_xx, P_xli],
                [P_mr, P_mli]]) @ np.block([[Hx(nu, i).T], 
                                            [Hl(nu, i).T]]) @ inv(Z)
    z = residual(convert_measurement_to_robot_frame(nu, y_meas), h_i(nu, i))

    nu_ = nu + K @ z    # Update state
    P_ = P - K @ Z @ K.T # Update covariance
    return nu_, P_, R_

def dist_from_point_to_line(point, endpoints):
    Px = endpoints[0,0]
    Py = endpoints[0,1]
    Qx = endpoints[1,0]
    Qy = endpoints[1,1]
    x = point[0]
    y = point[1]
    return np.abs((Qx - Px)*(Py - y) - (Px - x)*(Qy - Py))/(np.sqrt(np.power(Qx - Px, 2) + np.power(Qy - Py, 2)))

def dist_from_point_to_line_segment(point, endpoints):
    Px = endpoints[0,0]
    Py = endpoints[0,1]
    Qx = endpoints[1,0]
    Qy = endpoints[1,1]
    x = point[0]
    y = point[1,]
    t = -((Px - x)*(Qx - Px) + (Py - y)*(Qy - Py)) / (np.power(Px - Qx, 2) + np.power(Py - Qy, 2))
    if t >= 0 and t <= 1:
        # Point is perpendicular to the line segment from (Px, Py) to (Qx, Qy)
        return dist_from_point_to_line(point, endpoints)
    d1 = (np.sqrt(np.power(x - Px, 2) + np.power(y - Py, 2)))
    d2 = (np.sqrt(np.power(x - Qx, 2) + np.power(y - Qy, 2)))
    # Shortest distance from point to one of the endpoints
    if (d1 < d2):
        return d1
    return d2

def is_mergeable(line1, endpoints1, line2, endpoints2, angle_threshold, dist_threshold):
    phi1 = line1[1,0]
    phi2 = line2[1,0]
    angleDiff = np.arctan((np.tan(phi2) - np.tan(phi1)) / (1 + np.tan(phi1)*np.tan(phi2)))
    if (np.abs(angleDiff) > angle_threshold):
        # Not mergeable
        return 0
    
    # Is one of the endpoints of a line segment sifficiently near the other line segment?
    P1 = endpoints1[0]
    Q1 = endpoints1[1]
    dist1 = dist_from_point_to_line_segment(P1, endpoints2)
    dist2 = dist_from_point_to_line_segment(Q1, endpoints2)
    if (dist1 < dist_threshold or dist2 < dist_threshold):
        return 1
    return 0

def get_projected_point_on_line(r, phi, point):
    point_x = point[0]
    point_y = point[1]
    r_p = point_x*np.cos(phi) + point_y*np.sin(phi)
    d_x = np.abs(r_p - r)*np.cos(phi)
    d_y = np.abs(r_p- r)*np.sin(phi)
    if r_p - r >= 0:
        return np.array([[point_x - d_x],[point_y - d_y]])
    else:
        return np.array([[point_x + d_x],[point_y + d_y]])




'''y1 = np.array([[10], [0.0]])
endpoints1 = np.array([[10, -2], [10, 2]]) 

y2 = np.array([[10.5], [np.pi/2]])
endpoints2 = np.array([[-10, 10], [10, 10]]) 
DEG2RAD = np.pi / 180.0
ANGLE_THRES = 5 * DEG2RAD
DIST_THRES = 1
if is_mergeable(y1, endpoints1, y2, endpoints2, ANGLE_THRES, DIST_THRES):
    print("mergeable!")
else:
    print("not mergeable")

plot_line(y1[0], y1[1], endpoints1[0], endpoints2[1], color="black")
p1 = np.array([[12],[12]])
proj = get_projected_point_on_line(y1[0], y1[1], p1)
plt.plot(proj[0], proj[1], 'o', markersize=5, color="blue")
plt.plot(p1[0], p1[1], 'o', markersize=5, color="red")
plt.axis("equal")
plt.show()'''



# State vector, initially only holds robot pose
nu = np.array([
        [0],    # x
        [0],    # y
        [0]    # theta
    ]) 


# Control vector, linear velocity and rotational velocity
u = np.array([
        1.0, # v
        0.0 # w
        ])

dt = 1.0 # time-step

# Initial state covariance
P_ = np.eye(3,3) * np.array([[0, 0, 0]]) # sigma_x2, sigma_y2, sigma_theta2

# Process noise covariance
Q = np.eye(3,3) * np.array([[0.001, 0.001, 0.0001]]) # sigma_x2, sigma_y2, sigma_theta2

num_landmarks = 0

print("----- Init -------")
print(nu)
print(P_)
print("------------------")

track = []
track.append(nu)

plot_covariance_ellipse(
                    (nu[0,0], nu[1,0]), P_[0:2, 0:2], 
                     std=3, facecolor='k', alpha=0.3)

R = np.eye(2,2)*np.array([[0.1],[0.001]]) # measurement noise
l1 = np.array([[5], [np.pi/2 + np.pi]]) + np.sqrt(R) @ randn(2,1)
e1 = np.array([[0, -5], [10, -5]])
l2 = np.array([[10], [0.0]]) + np.sqrt(R) @ randn(2,1)
e2 = np.array([[10, -5], [10, 6]])
l3 = np.array([[6], [np.pi/2]])
e3 = np.array([[10, 6],[0, 6]])

landmarks = [l1, l2, l3]
endpoints = [e1, e2, e3]

for i in range(3):
    print(landmarks[i][0])
    print(landmarks[i][1])
    print(endpoints[i][0])
    print(endpoints[i][1])
    plot_line(landmarks[i][0], landmarks[i][1], endpoints[i][0], endpoints[i][1], color="black")

plt.axis("equal")
plt.show()


for step in range(10):
    if (step == 1):
        # Observe first landmark
        R = np.eye(2,2)*np.array([[0.1],[0.001]]) # measurement noise
        y = np.array([[10], [0.0]]) + np.sqrt(R) @ randn(2,1) # measurement
        endpoints = np.array([[10, -4], [10, 4]]) # Start and endpoint of line segment is not used in the filter (only used for data association and plotting)
        nu, P_, R = add_new_landmark(nu, P_, R, y, R)
       
        num_landmarks += 1
    if (step == 5):
        # Observe first landmark again
        R = np.eye(2,2)*np.array([[0.1],[0.001]]) # measurement noise
        y = np.array([[10], [0.0]]) + np.sqrt(R) @ randn(2,1) # measurement
        endpoints = np.array([[10, -2], [10, 2]])
        nu, P_, R = update(nu, P_, R, 0, y, R)
       

    nu, P_ = predict(nu, P_, u, dt, num_landmarks)

    plot_covariance_ellipse(
                    (nu[0,0], nu[1,0]), P_[0:2, 0:2], 
                     std=3, facecolor='k', alpha=0.3)
    track.append(nu[0:3])


track = np.array(track)

plt.plot(track[:, 0], track[:,1], '--', lw=1, color='black')

RAD2DEG = 180/np.pi
offsetAngle = 32
for i in track:
    plt.plot(i[0,0], i[1,0], marker=(3,1, i[2,0]*RAD2DEG+offsetAngle), markersize=15, color='green')

for i, line in enumerate(lines_global):
    r = line[0,0]
    phi = line[1,0]
    endpoints = endpoints_global[i]
    plot_line(r, phi, endpoints[0], endpoints[1])

plt.axis('equal')
plt.show()

'''



'''
######## Prediction #########
nu, P_ = predict(nu, P_, u, dt, num_landmarks)

print("------ Predict -------")
print(nu)
print(P_)


######## Update: Line feature observed #########
y0 = np.array([[10], [0.0]])            # Line feature measurement (NB: global frame is the "sensor frame")
R = np.eye(2,2)*np.array([[0.1],[0.001]]) # Measurement noise for feature 0
nu, P_, R = add_new_landmark(nu, P_, R, y0, R)
num_landmarks += 1

print("------ Observed line feature 0 -------")
print(nu)
print(P_)



######## Correction: Line feature 0 is observed again #########
i = 0
y0 = np.array([[11], [0.01]])    # Line feature measurement
R0 = np.eye(2,2)*np.array([[0.00001],[0.001]]) # Measurement noise for feature 0 this time

nu, P_, R, = update(nu, P_, R, i, y0, R0)

print("------ Observed line feature 0 again -------")
print(nu)
print(P_)
print("line 0 in global frame:", h_i(nu, i))

#S = H(nu) @ P_ @ H(nu).T + R1
#W = P_ @ H(nu).T @ inv(S)
#v = z1 - h_i(nu, 0)
#nu = nu + W @ v
#P_ = (np.eye(np.shape(P_)[0],np.shape(P_)[1]) - W @ H(nu)) @ P_










####### Update: new landmark, this is line feature 1 #################
y1 = np.array([[20], [np.pi/2]])    # Line feature
R1 = np.eye(2,2)*np.array([[0.1],[0.1]]) # Measurement noise for feature 1

nu, P_, R = add_new_landmark(nu, P_, R, y1, R1)
num_landmarks += 1

print("------ Observed line feature 1 -------")
print(nu)
print(P_)



####### Prediction #########
nu, P_ = predict(nu, P_, u, dt, num_landmarks)

print("------ Predict -------")
print(nu)
print(P_)




###### Prediction #########
nu, P_ = predict(nu, P_, u, dt, num_landmarks)

print("------ Predict -------")
print(nu)
print(P_)


###### Correction: Line feature 1 has been observed again #########
i = 1 # line feature index
y1 = np.array([[21], [np.pi/2 + 0.001]])    # Line feature
R3 = np.eye(2,2)*np.array([[0.1],[0.001]]) # Measurement noise for feature 0 this time

nu, P_, R = update(nu, P_, R, i, y1, R3)

#S = H(nu) @ P_ @ H(nu).T + R
#W = P_ @ H(nu).T @ inv(S)
#print(W)
#v = z3 - h_i(nu, 0)
#V = np.zeros((np.shape(nu)[0]-3, 1))
#V[0:2,:] = v
#nu = nu + W @ V
#P_ = (np.eye(np.shape(P_)[0],np.shape(P_)[1]) - W @ H(nu)) @ P_

print("------ Observed line feature 1 again -------")
print(nu)
print(P_)
print("line 1 in global frame:", h_i(nu, i))'''