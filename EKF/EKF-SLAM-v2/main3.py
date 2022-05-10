import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from filterpy.stats import plot_covariance_ellipse
from numpy.linalg import inv

def plot_line(r, phi, start, end, color='red'):
    if phi > np.pi/4:
        x = np.linspace(start[0],end[0], 10)
        y = (r - x*np.cos(phi)) / np.sin(phi)
    else:
        y = np.linspace(start[1], end[1], 10)
        x = (r - y*np.sin(phi)) / np.cos(phi)
    plt.plot(x, y, color = color)


# Inverse measurement model: Maps measurement to state
# This is used since we need to find some relationship between measurements and the robot pose 
def inv_h(x, l):
    r_G = l[0,0] # distance from origin of global coordinate system G to orthogonal projected point on line feature
    phi_G = l[1,0] # Angle between x-axis of G
    _x = x[0,0] # x position of robot
    _y = x[1,0] # y position of robot
    _theta = x[2,0] # robot heading
    r_R = r_G - (_x*np.cos(phi_G) + _y*np.sin(phi_G))
    phi_R = phi_G - _theta
    return np.array([[r_R],
                     [phi_R]])


# Measurement model for landmark i: Maps landmark i in state to measurement
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
                     [phi_G]])

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


def Hx(nu, i):
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
    return jacobian_hx

def Hl(nu, i):
    jacobian_hl = np.eye(2,2)
    landmark_index = 3 + i*2
    _x = nu[0,0]
    _y = nu[1,0]
    _theta = nu[2,0]
    phi_R = nu[landmark_index+1]
    jacobian_hl[0,1] = -_x*np.sin(phi_R+_theta) + _y*np.cos(phi_R + _theta)
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

def Gx(nu, y_G):
    phi_G = y_G[1,0]
    inv_meas_mod_jacobian_x = np.array([[-np.cos(phi_G), -np.sin(phi_G), 0],
                                      [0, 0, 1]])
    return inv_meas_mod_jacobian_x

def Gy(nu, y_G):
    phi_G = y_G[1,0]
    _x = nu[0,0]
    _y = nu[1,0]
    inv_meas_mod_jacobian_y = np.array([[1, _x*np.sin(phi_G) - _y*np.cos(phi_G)],
                                        [0, 1]])
    return inv_meas_mod_jacobian_y 

def update_measurement_noise_for_observation(prev_R, meas_noise, i):
    updated_R = prev_R
    updated_R[i*2:i*2+2, i*2:i*2+2] = meas_noise
    return updated_R
    
# State vector, initially only holds robot pose
nu = np.array([
        [0],    # x
        [0],    # y
        [0]    # theta
    ]) 

# Control vector, linear velocity and rotational velocity
u = np.array([
        1.0, # v
        0.1 # w
        ])

dt = 1.0 # time-step

# Initial state covariance
P_ = np.eye(3,3) * np.array([[0, 0, 0]]) # sigma_x2, sigma_y2, sigma_theta2

# Process noise covariance
Q = np.eye(3,3) * np.array([[0.00001, 0.00001, 0.00001]]) # sigma_x2, sigma_y2, sigma_theta2

num_landmarks = 0


######## Prediction #########
nu = f(nu, u, dt) # Predict state (no landmarks observed yet!)
P_ = F(nu, u, dt) @ P_ @ F(nu, u, dt).T + G(num_landmarks) @ Q @ G(num_landmarks).T

#print(P_)






######## Update: Line feature observed #########
z0 = np.array([[10], [0.0]])            # Line feature measurement (NB: global frame is the "sensor frame")
R = np.eye(2,2)*np.array([[0.1],[0.001]]) # Measurement noise for feature 0

# Map landmark to state using inverse measurement model
l0 = inv_h(nu, z0)
nu = add_new_landmark_to_state(nu, l0) # Append new landmark to state vector

# Initial landmark covariance
P_ll = Gx(nu, z0) @ P_ @ Gx(nu, z0).T + Gy(nu, z0) @ R @ Gy(nu, z0).T
# Initial landmark cross-covariance
P_lx = Gx(nu, z0) @ P_[0:3, 0:3]
# Append initial landmark covariance matrix to full-state covariance matrix
P_ = np.block([[P_, P_lx.T], 
               [P_lx, P_ll]])

num_landmarks += 1







######## Correction: Line feature 0 is observed again #########
z1 = np.array([[11], [0.01]])    # Line feature
R1 = np.eye(2,2)*np.array([[0.1],[0.001]]) # Measurement noise for feature 0 this time

S = H(nu) @ P_ @ H(nu).T + R1
W = P_ @ H(nu).T @ inv(S)
v = z1 - h_i(nu, 0)
nu = nu + W @ v
P_ = (np.eye(np.shape(P_)[0],np.shape(P_)[1]) - W @ H(nu)) @ P_










####### Update: new landmark, this is line feature 1 #################
z2 = np.array([[20], [np.pi/2]])    # Line feature
R2 = np.eye(2,2)*np.array([[0.1],[0.1]]) # Measurement noise for feature 1

# Map landmark to state using inverse measurement model
l2 = inv_h(nu, z2)
nu = add_new_landmark_to_state(nu, l2)

# Initial landmark covariance
P_ll = Gx(nu, z2) @ P_[0:3, 0:3] @ Gx(nu, z2).T + Gy(nu, z2) @ R2 @ Gy(nu, z2).T
# Initial landmark cross-covariance
P_lx = Gx(nu, z2) @ P_[0:3, :]
# Update full-state covariance
P_ = np.block([[P_, P_lx.T], 
               [P_lx, P_ll]])

R = add_new_landmark_covariance(R, R2)
num_landmarks += 1





####### Prediction #########
nu = f(nu, u, dt)
P_ = F(nu, u, dt) @ P_ @ F(nu, u, dt).T + G(num_landmarks) @ Q @ G(num_landmarks).T






###### Prediction #########
nu = f(nu, u, dt)
P_ = F(nu, u, dt) @ P_ @ F(nu, u, dt).T + G(num_landmarks) @ Q @ G(num_landmarks).T




###### Correction: Line feature 0 has been observed again #########
i = 0 # update line feature index
z3 = np.array([[11], [0.01]])    # Line feature
R3 = np.eye(2,2)*np.array([[0.1],[0.001]]) # Measurement noise for feature 0 this time

R = update_measurement_noise_for_observation(R, R3, 0)

P_xx = P_[0:3,0:3]
P_xl0 = P_[0:3+i*2,3+i*2:3+i*2+2]
P_ll0 = P_[3+i*2:3+i*2+2, 3+i*2:3+i*2+2]


# computing the innovation is sparse: only take care of the robot state, the concerned landmark and the robot-landmark i covariances
Z = np.block([[Hx(nu,i), Hl(nu, i)]]) @ np.block([[P_xx, P_xl0],
                                                  [P_xl0.T, P_ll0]]) @ np.block([[Hx(nu, i).T], 
                                                                                 [Hl(nu, i).T]]) + R[i*2:i*2+2, i*2:i*2+2]

print(np.block([[Hx(nu, i).T],[Hl(nu, i).T]]) @ inv(Z))
P_mr = P_[3:,0:3]
P_ml0 = None # Need to find this inside P_

#K = np.block([[P_xx, P_xl0],
#              [P_mr, P_ml0]]) @ np.block([[Hx(nu, i).T], 
#                                          [Hl(nu, i).T]]) @ inv(Z)



#S = H(nu) @ P_ @ H(nu).T + R
#W = P_ @ H(nu).T @ inv(S)
#print(W)
#v = z3 - h_i(nu, 0)
#V = np.zeros((np.shape(nu)[0]-3, 1))
#V[0:2,:] = v
#nu = nu + W @ V
#P_ = (np.eye(np.shape(P_)[0],np.shape(P_)[1]) - W @ H(nu)) @ P_

#print(P_)